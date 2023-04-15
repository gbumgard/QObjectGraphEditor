#include "TemporalMedianFilter.h"
#include "ObjectModel.h"
#include <QMutexLocker>
#include <QFuture>
#include <QtConcurrent/QtConcurrent>

#include <opencv2/opencv.hpp>

#include <QDebug>

REGISTER_CLASS(TemporalMedianFilter)

TemporalMedianFilter::TemporalMedianFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _aperatureSize(0)
{
}

void TemporalMedianFilter::aperatureSize(int aperatureSize) {
  if (aperatureSize < 0) aperatureSize = 0;
  else if (aperatureSize > 2) aperatureSize = 2;
  _aperatureSize = aperatureSize;
  _frameBuffer.clear();
}

void TemporalMedianFilter::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);

    cv::Mat mat(matEvent.mat());

    if (!_frameBuffer.empty() &&
        (mat.rows != _frameBuffer[0].rows || mat.cols != _frameBuffer[0].cols)) {
        _frameBuffer.clear();
    }

    qDebug() << "input" << mat;

    _frameBuffer.push_back(mat.clone());

    size_t kernelSize = (_aperatureSize + 1) * 2 + 1;

    if (_frameBuffer.size() < kernelSize) {
        while (_frameBuffer.size() < kernelSize) {
            _frameBuffer.push_back(mat.clone());
        }
    }
    else {
        while (_frameBuffer.size() > kernelSize) {
            _frameBuffer.erase(_frameBuffer.begin());
        }
    }

    if (_task.isRunning()) return;

    if (_task.isValid()) {
        emit out(QVariant::fromValue(MatEvent(_task.takeResult())));
    }

    _task = QtConcurrent::run([&]() {
        std::vector<cv::Mat> frameBuffer(_frameBuffer);
        size_t center = _frameBuffer.size() / 2;
        cv::Mat median(frameBuffer[0].rows, frameBuffer[0].cols, CV_8UC1);
        qDebug() << "median" << median;
        for (int x = 0; x < median.cols; x++) {
            for (int y = 0; y < median.rows; y++) {
                std::vector<uint8_t> values;
                for (auto frame : frameBuffer) {
                    values.push_back(frame.at<uint8_t>(y,x));
                }
                std::sort(values.begin(), values.end());
                median.at<uint8_t>(y,x) = values[center];
            }
        }
        return median;
    });
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
