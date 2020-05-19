#include "ThresholdFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ThresholdFilter)

ThresholdFilter::ThresholdFilter(QObject *parent)
  : QObject(parent)
  , _filterType(THRESH_BINARY)
  , _threshold(0.0)
  , _maximum(255.0)
  , _logcat("ThresholdFilter")
{
  qCDebug(_logcat) << Q_FUNC_INFO;
  _logcat.setEnabled(QtDebugMsg,false);
  _logcat.setEnabled(QtInfoMsg,false);
}

void ThresholdFilter::in(const cv::Mat &mat) {
  qCDebug(_logcat) << Q_FUNC_INFO;
  _input = mat;
  update();
}

void ThresholdFilter::setFilterType(FilterType filterType) {
  qCDebug(_logcat) << Q_FUNC_INFO << filterType;
  _filterType = filterType;
  update();
}

void ThresholdFilter::threshold(double threshold) {
  qCDebug(_logcat) << Q_FUNC_INFO << threshold;
  _threshold = threshold;
  update();
}

void ThresholdFilter::maximum(double maximum) {
  qCDebug(_logcat) << Q_FUNC_INFO << maximum;
  _maximum = maximum;
  update();
}

void ThresholdFilter::update() {
  qCDebug(_logcat) << Q_FUNC_INFO << _threshold << _maximum << _filterType;
  cv::Mat dst;
  cv::threshold(_input,dst,_threshold,_maximum, _filterType);
  emit out(dst);
}
