#include "MatCompare.h"
#include "ObjectModel.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#define C1 (float) (0.01 * 255 * 0.01  * 255)
#define C2 (float) (0.03 * 255 * 0.03  * 255)

REGISTER_CLASS(MatCompare)

MatCompare::MatCompare(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _keyFrameInterval(30)
    , _blockSize(1)
    , _enableMask(false)
    , _filterWeight(1.0)
    , _filteredError(0.0)
    , _errorThreshold(0)
    , _frameCount(0)
    , _reference()
{
}

void MatCompare::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat grayscale;
        if (matEvent.mat().channels() == 3) {
            //input image is single channel, grayscale
            cvtColor(matEvent.mat(), grayscale, cv::COLOR_RGB2GRAY);
        }
        else {
            grayscale = matEvent.mat();
        }
        cv::Mat mat;
        grayscale.convertTo(mat,CV_8U);
        if (!_reference.empty()) {
            double rmseVal = rmse(mat, _reference, _enableMask);
            double psnrVal = cv::PSNR(mat,_reference);
            _filteredError = _filteredError * (1.0 - _filterWeight) + _filterWeight * psnrVal;
            //double absdiff = abs(rmseVal - _filteredError);
            double psnrDiff = abs(psnrVal - _filteredError);
            //double _ssim = ssim(dmat, _reference, _blockSize, _enableMask);
            //double _psnr = psnr(dmat, _reference, _blockSize);

            //qDebug() << "RMSE: " << _rmse << " SSIM: " << _ssim << " PSNR: " << _psnr;
            //qDebug() << "RMSE: " << rmseVal << " Filtered RMSE: " << _filteredError << " Diff: " << abs(rmseVal - _filteredError) << " PSNR: " << psnrVal;
            //qDebug("RMSE: %2.3lf  Filtered RMSE: %2.3lf  Diff: %2.3lf  PSNR: %2.3lf", rmseVal, _filteredError, psnrDiff, psnrVal);
            qDebug("PSNR: %2.3lf  Filtered PSNR: %2.3lf  Diff: %2.3lf", psnrVal, _filteredError, psnrDiff);

            emit rmse(rmseVal);
            //emit ssim(_ssim);
            emit psnr(psnrVal);
            emit diff(psnrDiff);
            cv::Mat diffMat;
            cv::absdiff(matEvent.mat(),_reference,diffMat);
            if (psnrDiff > _errorThreshold) {

                emit absDiff(QVariant::fromValue(MatEvent(diffMat,matEvent.timestamp())));
                _reference = mat;
                if (_frameCount != 0) _frameCount = 1;
            }
            else if (_frameCount != 0) {
                _frameCount++;
                if (_frameCount >= _keyFrameInterval) {
                    _reference = mat;
                    _frameCount = 1;
                }
            }
            emit out(QVariant::fromValue(MatEvent(_reference,matEvent.timestamp())));
        }
        else {
            _reference = mat;
        }
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

void MatCompare::ref(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat grayscale;
        if (matEvent.mat().channels() == 3) {
            //input image is single channel, grayscale
            cvtColor(matEvent.mat(), grayscale, cv::COLOR_RGB2GRAY);
        }
        else {
            grayscale = matEvent.mat();
        }
        grayscale.convertTo(_reference,CV_8U);
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

double MatCompare::sigma(cv::Mat & m, int i, int j, int block_size) {

    double sd = 0;

    cv::Mat m_tmp = m(cv::Range(i, i + block_size), cv::Range(j, j + block_size));
    cv::Mat m_squared(block_size, block_size, CV_64F);

    cv::multiply(m_tmp, m_tmp, m_squared);

    // E(x)
    double avg = mean(m_tmp)[0];
    // E(x�)
    double avg_2 = mean(m_squared)[0];

    sd = sqrt(avg_2 - avg * avg);

    return sd;
}

// Covariance
double MatCompare::cov(cv::Mat & m0, cv::Mat & m1, int i, int j, int block_size) {
    cv::Mat m3 = cv::Mat::zeros(block_size, block_size, m0.depth());
    cv::Mat m0_tmp = m0(cv::Range(i, i + block_size), cv::Range(j, j + block_size));
    cv::Mat m1_tmp = m1(cv::Range(i, i + block_size), cv::Range(j, j + block_size));

    cv::multiply(m0_tmp, m1_tmp, m3);

    double avg_ro = cv::mean(m3)[0]; // E(XY)
    double avg_r = cv::mean(m0_tmp)[0]; // E(X)
    double avg_o = cv::mean(m1_tmp)[0]; // E(Y)

    double sd_ro = avg_ro - avg_o * avg_r; // E(XY) - E(X)E(Y)
    return sd_ro;
}

// Mean squared error
double MatCompare::mse(cv::Mat & m0, cv::Mat & m1, bool grayscale, bool rooted, bool enableMask) {
    double res = 0;
    int H = m0.rows, W = m0.cols, blanks = 0;
    for (int i = 0; i < H; i++)
        for (int j = 0; j < W; j++) {
            if (grayscale) {
                double p0 = m0.at<uchar>(i, j), p1 = m1.at<uchar>(i, j);
                if (enableMask) {
                    if ((p0 > 254.0) || (p0 < 1.0)) {
                        ++blanks;
                        continue;
                    }
                }
                double diff = cv::abs(p0 - p1);
                res += diff * diff;
            }
            else {
                cv::Vec3b p0 = m0.at<cv::Vec3b>(i, j);
                cv::Vec3b p1 = m1.at<cv::Vec3b>(i, j);
                if (enableMask) {
                    if ((p0.val[0] > 254 && p0.val[1] > 254 && p0.val[2] > 254) || (p1.val[0] < 1 && p1.val[1] < 1 && p1.val[2] < 1)) {
                        ++blanks;
                        continue;
                    }
                }
                double d0 = cv::abs(p0.val[0] - p1.val[0]);
                double d1 = cv::abs(p0.val[1] - p1.val[1]);
                double d2 = cv::abs(p0.val[2] - p1.val[2]);
                if (rooted) {
                    res += sqrt(d0 * d0 + d1 * d1 + d2 * d2) / 255.0 / sqrt(3.0);
                }
                else {
                    res += (d0 * d0 + d1 * d1 + d2 * d2) / 3.0;
                }
            }
        }
    res /= H * W - blanks;
    return res;
}

// Rooted mean squared error
double MatCompare::rmse(cv::Mat & m0, cv::Mat & m1, bool) {
    cv::Mat diff = m0 - m1;
    cv::pow(diff,2,diff);
    double totalError = cv::sum(diff)[0];
    double mse = totalError/(m0.rows * m0.cols);
    double _rmse = sqrt(mse);
    return _rmse;
#if 0
    double eqm = 0;
    int height = m0.rows;
    int width = m0.cols;
    int blanks = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (enableMask) {
                cv::Vec3b p0 = m0.at<cv::Vec3b>(i, j);
                cv::Vec3b p1 = m1.at<cv::Vec3b>(i, j);
                if ((p0.val[0] > 254 && p0.val[1] > 254 && p0.val[2] > 254) || (p1.val[0] < 1 && p1.val[1] < 1 && p1.val[2] < 1)) {
                    ++blanks;
                    continue;
                }
            }
            //double diff = cv::abs(p0.val[0] - p1.val[0]) + cv::abs(p0.val[1] - p1.val[1]) + cv::abs(p0.val[2] - p1.val[2]);
            double p0 = m0.at<double>(i, j);
            double p1 = m1.at<double>(i, j);
            double diff = p1 - p0;
            eqm += diff * diff;
        }
    }
    double _rmse = sqrt(eqm / (height * width - blanks));

    return _rmse;
#endif
}


// Peak signal-to-noise ratio
// https://en.wikipedia.org/wiki/Peak_signal-to-noise_ratio
double MatCompare::psnr(cv::Mat & m0, cv::Mat & m1, int) {
    int D = 255;
    return (10 * log10((D*D) / mse(m0, m1, true, false)));
}

// Structural Similarity Index Measure
double MatCompare::ssim(cv::Mat & m0, cv::Mat & m1, int block_size, bool enableMask) {
    double ssim = 0;

    int nbBlockPerHeight = m0.rows / block_size;
    int nbBlockPerWidth = m0.cols / block_size;
    double ssim_total = 0;

    for (int k = 0; k < nbBlockPerHeight; k++) {
        for (int l = 0; l < nbBlockPerWidth; l++) {
            int m = k * block_size;
            int n = l * block_size;

            double avg_o = cv::mean(m0(cv::Range(k, k + block_size), cv::Range(l, l + block_size)))[0];
            double avg_r = cv::mean(m1(cv::Range(k, k + block_size), cv::Range(l, l + block_size)))[0];
            if (enableMask) {
                if (avg_o > 254 || avg_o < 1) {
                    continue;
                }
                ssim_total += 1;
            }
            double sigma_o = sigma(m0, m, n, block_size);
            double sigma_r = sigma(m1, m, n, block_size);
            double sigma_ro = cov(m0, m1, m, n, block_size);

            ssim += ((2 * avg_o * avg_r + C1) * (2 * sigma_ro + C2)) / ((avg_o * avg_o + avg_r * avg_r + C1) * (sigma_o * sigma_o + sigma_r * sigma_r + C2));
        }
    }

    ssim /= enableMask ? ssim_total : nbBlockPerHeight * nbBlockPerWidth;
    return ssim;
}

