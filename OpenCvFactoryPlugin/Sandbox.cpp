#include "Sandbox.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

static cv::Mat getStructuringElement(cv::MorphShapes shape, int kernelSize, cv::Point anchor)
{
    int i, j;
    int r = 0, c = 0;
    double inv_r2 = 0;

    cv::Size ksize(kernelSize,kernelSize);

    CV_Assert( shape == cv::MORPH_RECT || shape == cv::MORPH_CROSS || shape == cv::MORPH_ELLIPSE );

    if (anchor.x < 0) anchor.x = kernelSize >> 1;
    if (anchor.y < 0) anchor.y = kernelSize >> 1;

    if( ksize == cv::Size(1,1) )
        shape = cv::MORPH_RECT;

    if( shape == cv::MORPH_ELLIPSE )
    {
        r = ksize.height/2;
        c = ksize.width/2;
        inv_r2 = r ? 1./((double)r*r) : 0;
    }

    cv::Mat elem(ksize, CV_8U);

    for( i = 0; i < ksize.height; i++ )
    {
        uchar* ptr = elem.ptr(i);
        int j1 = 0, j2 = 0;

        if( shape == cv::MORPH_RECT || (shape == cv::MORPH_CROSS && i == anchor.y) )
            j2 = ksize.width;
        else if( shape == cv::MORPH_CROSS )
            j1 = anchor.x, j2 = j1 + 1;
        else
        {
            int dy = i - r;
            if( std::abs(dy) <= r )
            {
                int dx = cv::saturate_cast<int>(c*std::sqrt((r*r - dy*dy)*inv_r2));
                j1 = std::max( c - dx, 0 );
                j2 = std::min( c + dx + 1, ksize.width );
            }
        }

        for( j = 0; j < j1; j++ )
            ptr[j] = 0;
        for( ; j < j2; j++ )
            ptr[j] = 1;
        for( ; j < ksize.width; j++ )
            ptr[j] = 0;
    }

    return elem;
}

REGISTER_CLASS(Sandbox)

Sandbox::Sandbox(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _min(500) // millimeters
    , _max(1000) // millimeters
    , _filterWeight(1.0)
    , _bkgndSubtractor(cv::bgsegm::createBackgroundSubtractorCNT())
    , _slowFilterWeight(.05)
    , _fastFilterWeight(.80)
    , _filterThreshold(1)
{
    _useHistory = _bkgndSubtractor->getUseHistory();
    _minPixelStability = _bkgndSubtractor->getMinPixelStability();
    _maxPixelStability = _bkgndSubtractor->getMaxPixelStability();
}

void Sandbox::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat input(matEvent.mat());
        double inputRange = _max - _min;
        double scale = 255.0 / inputRange;
        cv::Mat aboveLowerMask = input >= _min;
        cv::Mat belowUpperMask = input <= _max;
        cv::Mat inRangeMask = aboveLowerMask & belowUpperMask;
        cv::Mat output = input.clone();
        output -= _min;
        output *= scale;
        output.copyTo(output, inRangeMask);
        cv::convertScaleAbs(output, output, -1.0, 255);
        cv::threshold(output, output, 254, 255, cv::THRESH_TOZERO_INV);
        emit norm(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
        _bkgndSubtractor->setMinPixelStability(_minPixelStability);
        _bkgndSubtractor->setMaxPixelStability(_maxPixelStability);
        _bkgndSubtractor->setUseHistory(_useHistory);
        cv::Mat fgndMask;
        _bkgndSubtractor->apply(output, fgndMask, _filterWeight);
        emit mask(QVariant::fromValue(MatEvent(fgndMask,matEvent.timestamp())));
        cv::Mat fgndMat;
        output.copyTo(fgndMat,fgndMask);
        emit fgnd(QVariant::fromValue(MatEvent(fgndMat,matEvent.timestamp())));
        cv::Mat bgndMat;
        output.copyTo(bgndMat,~fgndMask);
        emit bgnd(QVariant::fromValue(MatEvent(bgndMat,matEvent.timestamp())));
        cv::Mat bkgimage;
        _bkgndSubtractor->getBackgroundImage(bkgimage);
        emit out(QVariant::fromValue(MatEvent(bkgimage,matEvent.timestamp())));

        if (!_frameBuffer.empty() &&
            (output.rows != _frameBuffer[0].rows || output.cols != _frameBuffer[0].cols)) {
            _frameBuffer.clear();
        }

        size_t kernelSize =3;

        if (_frameBuffer.size() < kernelSize) {
            while (_frameBuffer.size() < kernelSize) {
                _frameBuffer.push_back(output.clone());
            }
        }

        cv::Mat sum(_frameBuffer[0].rows,_frameBuffer[0].cols,CV_32F,cv::Scalar(0));
        for (auto& frame : _frameBuffer) {
            sum += frame;
        }

        cv::Mat sum2(_frameBuffer[0].rows,_frameBuffer[0].cols,CV_32F,cv::Scalar(0));
        for (auto& frame : _frameBuffer) {

            cv::Mat square;
            cv::accumulateSquare(frame,sum2);
        }

        cv::Mat average = sum / _frameBuffer.size();

        cv::Mat ave2(_frameBuffer[0].rows,_frameBuffer[0].cols,CV_32F,cv::Scalar(0));
        cv::accumulateSquare(average,ave2);

        cv::cvtColor(average,average,CV_8UC1);
        emit mean(QVariant::fromValue(MatEvent(average,matEvent.timestamp())));

        cv::Mat variance = (sum2 / _frameBuffer.size()) - ave2;
        //cv::cvtColor(variance,variance,CV_8UC1);

        emit var(QVariant::fromValue(MatEvent(variance,matEvent.timestamp())));

        _frameBuffer.push_back(output.clone());

        while (_frameBuffer.size() > kernelSize) {
        _frameBuffer.erase(_frameBuffer.begin());
        }

        int alpha = 0;
        switch (input.depth()) {
            case CV_8U:
            case CV_8S:
            case CV_16U:
            case CV_16S:
                alpha = CV_32F;
                break;
            case CV_32S:
            case CV_32F:
            default:
                alpha = CV_64F;
            break;
        }
        int type = CV_MAKETYPE(alpha,input.channels());

        if (_slowAccumulator.empty() ||
            input.channels() != _slowAccumulator.channels() ||
            input.size() != _slowAccumulator.size()) {

            _slowAccumulator = cv::Mat(input.size(), type, cv::Scalar(0.0));
            _stabilityCount= cv::Mat(input.size(), CV_16UC1, cv::Scalar(0));
            _output= cv::Mat(input.size(), CV_8UC1, cv::Scalar(0));
        }

        if (_fastAccumulator.empty() ||
            input.channels() != _fastAccumulator.channels() ||
            input.size() != _fastAccumulator.size()) {

            _fastAccumulator = cv::Mat(input.size(), type, cv::Scalar(0.0));
        }

#if 0
        cv::accumulateWeighted(bkgimage, _slowAccumulator, _slowFilterWeight);
        cv::accumulateWeighted(bkgimage, _fastAccumulator, _fastFilterWeight);
#else
        cv::accumulateWeighted(output, _slowAccumulator, _slowFilterWeight);
        cv::accumulateWeighted(output, _fastAccumulator, _fastFilterWeight);
#endif
        cv::Mat absDiff;
        cv::absdiff(_slowAccumulator,_fastAccumulator,absDiff);
        cv::Mat filterMask = absDiff > _filterThreshold;
        emit fmask(QVariant::fromValue(MatEvent(filterMask,matEvent.timestamp())));
        cv::Mat zeros = cv::Mat::zeros(input.size(),CV_16U);
        zeros.copyTo(_stabilityCount,filterMask);
        cv::Mat ones = cv::Mat::ones(input.size(),CV_16U);
        cv::Mat updateMask = (_stabilityCount < _minPixelStability);
        cv::copyTo(updateMask,updateMask,~filterMask);
        cv::add(_stabilityCount,ones,_stabilityCount,updateMask);
        //cv::threshold(absDiff, filterMask, _filterThreshold, 255, cv::THRESH_BINARY);
        emit umask(QVariant::fromValue(MatEvent(updateMask,matEvent.timestamp())));
        emit stability(QVariant::fromValue(MatEvent(_stabilityCount,matEvent.timestamp())));
        cv::Mat dispMask = _stabilityCount >= (_minPixelStability - 3);
        emit dmask(QVariant::fromValue(MatEvent(dispMask.clone(),matEvent.timestamp())));

        cv::accumulateWeighted(output, _slowAccumulator, 1.0, filterMask);
        cv::Mat smoothed;
        _slowAccumulator.convertTo(smoothed,CV_8U);
        emit smooth(QVariant::fromValue(MatEvent(smoothed,matEvent.timestamp())));

        if (_output.empty() ||
            input.channels() != _output.channels() ||
            input.size() != _output.size()) {

            _output= cv::Mat(input.size(), CV_8UC1, cv::Scalar(0));
        }
        cv::Mat structuringElement = getStructuringElement(cv::MORPH_ELLIPSE, 3, cv::Point(-1,-1));

        cv::morphologyEx(dispMask,dispMask,cv::MORPH_CLOSE,structuringElement,cv::Point(-1,-1),1);

        cv::copyTo(smoothed,_output,dispMask);
        emit final(QVariant::fromValue(MatEvent(_output,matEvent.timestamp())));

    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported input type");
    }
}
