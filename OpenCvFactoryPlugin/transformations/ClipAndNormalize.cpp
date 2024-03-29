#include "ClipAndNormalize.h"
#include "ObjectModel.h"

REGISTER_CLASS(ClipAndNormalize)

ClipAndNormalize::ClipAndNormalize(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _minimum(0)
  , _maximum(255)
  , _range(255)
  , _offset(0)
{
}

void ClipAndNormalize::min(double newMin) {
  if (newMin != _minimum) {
    _minimum = newMin;
    emit minChanged(newMin);
  }
}

void ClipAndNormalize::max(double newMax) {
  if (newMax != _maximum) {
    _maximum = newMax;
    emit minChanged(newMax);
  }
}

void ClipAndNormalize::range(double newRange) {
  _range = newRange;
}

void ClipAndNormalize::offset(double newOffset) {
  _offset = newOffset;
}

void ClipAndNormalize::src(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);
    double inputRange = _maximum - _minimum;
    double scale = _range / inputRange;
    cv::Mat aboveLowerMask = matEvent.mat() >= _minimum;
    cv::Mat belowUpperMask = matEvent.mat() <= _maximum;
    cv::Mat inRangeMask = aboveLowerMask & belowUpperMask;
    cv::Mat normalized = matEvent.mat().clone();
    normalized -= _minimum;
    normalized *= scale;
    normalized += _offset;
    cv::Mat output;
    normalized.copyTo(output,inRangeMask);
    emit dst(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
