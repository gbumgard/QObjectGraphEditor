#include "ThresholdFilter.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ThresholdFilter)

ThresholdFilter::ThresholdFilter(QObject *parent)
  : AbstractOpenCvObject(parent)
  , _filterType(THRESH_BINARY)
  , _threshold(0.0)
  , _maximum(255.0)
  , _logcat("TaggedThresholdFilter")
{
  qCDebug(_logcat) << Q_FUNC_INFO;
  _logcat.setEnabled(QtDebugMsg,false);
  _logcat.setEnabled(QtInfoMsg,false);
}

void ThresholdFilter::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);
      cv::Mat output;
      cv::threshold(matEvent.mat(),output,_threshold,_maximum, _filterType);
      emit out(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
  }
  else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
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

