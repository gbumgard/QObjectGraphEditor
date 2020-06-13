#include "ThresholdFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ThresholdFilter)

ThresholdFilter::ThresholdFilter(QObject *parent)
  : QObject(parent)
  , _filterType(THRESH_BINARY)
  , _threshold(0.0)
  , _maximum(255.0)
  , _logcat("TaggedThresholdFilter")
{
  qCDebug(_logcat) << Q_FUNC_INFO;
  _logcat.setEnabled(QtDebugMsg,false);
  _logcat.setEnabled(QtInfoMsg,false);
}

void ThresholdFilter::in(const TaggedMat &taggedMat) {
  qCDebug(_logcat) << Q_FUNC_INFO;
  cv::Mat dst;
  cv::threshold(taggedMat.first,dst,_threshold,_maximum, _filterType);
  emit out(TaggedMat(dst,taggedMat.second));
}

void ThresholdFilter::setFilterType(FilterType filterType) {
  qCDebug(_logcat) << Q_FUNC_INFO << filterType;
  _filterType = filterType;
}

void ThresholdFilter::threshold(double threshold) {
  qCDebug(_logcat) << Q_FUNC_INFO << threshold;
  _threshold = threshold;
}

void ThresholdFilter::maximum(double maximum) {
  qCDebug(_logcat) << Q_FUNC_INFO << maximum;
  _maximum = maximum;
}

