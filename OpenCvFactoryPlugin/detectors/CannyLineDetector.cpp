#include "detectors/CannyLineDetector.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(CannyLineDetector)

CannyLineDetector::CannyLineDetector(QObject* parent)
  : ThreadedObject(parent)
  , _nextImage()
  , _threshold(1.0)
  , _ratio(.3)
  , _aperatureSize(APERATURE_3X3)
  , _enableL2Gradient(false)
{
  start();
}

CannyLineDetector::~CannyLineDetector() {
  _nextImage = cv::Mat();
}

void CannyLineDetector::threshold(double value) {
  UpdateLock lock(this);
  _threshold = value;
}

void CannyLineDetector::ratio(double value) {
  UpdateLock lock(this);
  _ratio = value;
}

void CannyLineDetector::aperatureSize(AperatureSize aperature) {
  UpdateLock lock(this);
  _aperatureSize = aperature;
}

void CannyLineDetector::enableL2Gradient(bool value) {
  UpdateLock lock(this);
  _enableL2Gradient = value;
}

void CannyLineDetector::in(const cv::Mat& image) {
  UpdateLock lock(this);
  _nextImage = image;
}

void CannyLineDetector::update() {
  cv::Mat currentImage;
  double currentThreshold;
  double currentRatio;
  AperatureSize currentAperatureSize;
  bool currentEnableL2Gradient;
  {
    WaitForUpdateLock lock(this);
    if (_stop) return;
    currentImage = _nextImage;
    _nextImage = cv::Mat();
    if (!currentImage.empty()) {
      currentThreshold = _threshold;
      currentRatio = _ratio;
      currentAperatureSize = _aperatureSize;
      currentEnableL2Gradient = _enableL2Gradient;
    }
    else {
      return;
    }
  }

  cv::Mat dst(cv::Size(currentImage.cols, currentImage.rows), CV_8UC1);
  cv::Canny(currentImage, dst, currentThreshold, currentRatio * currentThreshold, (int)currentAperatureSize, currentEnableL2Gradient);
  emit out(dst);
}
