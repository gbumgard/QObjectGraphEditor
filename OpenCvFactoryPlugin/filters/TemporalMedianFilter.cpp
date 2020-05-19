#include "TemporalMedianFilter.h"
#include <QMutexLocker>
#include "OpenCvFactoryPlugin.h"

#include <opencv2/opencv.hpp>

#include <QDebug>

REGISTER_CLASS(TemporalMedianFilter)

TemporalMedianFilter::TemporalMedianFilter(QObject* parent)
  : ThreadedObject(parent)
  , _aperatureSize(3)
  , _width(0)
  , _height(0)
{
  start();
}

void TemporalMedianFilter::aperatureSize(int aperatureSize) {
  UpdateLock lock(this);
  if (aperatureSize < 3) aperatureSize = 3;
  else if (aperatureSize > 7) aperatureSize = 7;
  _aperatureSize = 1 + (aperatureSize / 2) * 2;
  qDebug() << Q_FUNC_INFO << "aperature size" << _aperatureSize;
  while(_inputBuffer.size() > (unsigned)_aperatureSize) {
    _inputBuffer.erase(_inputBuffer.begin());
  }
}

void TemporalMedianFilter::in(const cv::Mat &mat) {
  qDebug() << Q_FUNC_INFO << "wait";
  UpdateLock lock(this);
  qDebug() << Q_FUNC_INFO << "continue";
  if (mat.rows != _height || mat.cols != _width) {
    qDebug() << Q_FUNC_INFO << "clear buffer for size change";
    _inputBuffer.clear();
    _height = mat.rows;
    _width = mat.cols;
  }

  _inputBuffer.push_back(mat.clone());

  while (_inputBuffer.size() > (unsigned)_aperatureSize) {
    qDebug() << Q_FUNC_INFO << "erase first image to match aperature size" << _inputBuffer.size() << _aperatureSize;
    _inputBuffer.erase(_inputBuffer.begin());
  }
}

void TemporalMedianFilter::update() {
  qDebug() << Q_FUNC_INFO;
  std::vector<cv::Mat> currentImages;
  int currentAperatureSize;
  int currentWidth, currentHeight;
  {
    WaitForUpdateLock lock(this);
    qDebug() << Q_FUNC_INFO << "awake buffer depth:" << _inputBuffer.size() << "aperature:" << _aperatureSize;
    if (_stop) return;
    for (auto iter = _inputBuffer.begin(); iter != _inputBuffer.end(); iter++) {
      currentImages.push_back(*iter);
    }
    currentAperatureSize = _aperatureSize;
    currentWidth = _width;
    currentHeight = _height;
  }

  if (currentImages.size() > (unsigned)currentAperatureSize / 2) {
    if (currentImages.size() >= (unsigned)currentAperatureSize) {
      cv::Mat median(currentHeight, currentWidth, CV_8UC1, cv::Scalar(DBL_MAX));
      int* values = new int[currentAperatureSize];
      for (int x = 0; x < currentWidth; x++) {
        for (int y = 0; y < currentHeight; y++) {
          for (int z = 0; z < currentAperatureSize; ++z)
          {
            *(values + z) = currentImages.at(z).at<uint8_t>(y,x);
          }
          std::sort(values, values+currentAperatureSize);
          int center = currentAperatureSize / 2;
          median.at<uint8_t>(y,x) = currentAperatureSize % 2 ? *(values + center)
                                                       : (*(values + center - 1) + *(values + center)) / 2;
        }
      }
      emit out(median);
    }
    else {
      emit out(currentImages[currentAperatureSize/2]);
    }
  }
}
