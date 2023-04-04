#include "AccumulateWeighted.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AccumulateWeighted)

AccumulateWeighted::AccumulateWeighted(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _alpha(1.0)
  , _srcEvent(cv::Mat(),MatEvent::UNTIMED)
  , _maskEvent(cv::Mat(),MatEvent::UNTIMED)
{
}

void AccumulateWeighted::alpha(double alpha) {
  if (alpha != _alpha) {
    _alpha = alpha < 0.0 ? 0.0 : alpha > 1.0 ? 1.0 : alpha;
  }
  if (!_accumulator.empty()) _accumulator = 0.0;
}

void AccumulateWeighted::src(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    _srcEvent = qvariant_cast<MatEvent>(variant);
    // Check to see if _src1 and _src2 have the same attributes and are synchronized
    if (_maskEvent && !_srcEvent.isSynchronizedMask(_maskEvent)) {
      // The current _mask cannot be used with _src1 so we release it.
      _maskEvent.release();
      return;
    }
    updateObject();
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}

void AccumulateWeighted::mask(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    _maskEvent = qvariant_cast<MatEvent>(variant);
    // Check to see if _src1 and _src2 have the same attributes and are synchronized
    if (_maskEvent.isSynchronizedMask(_srcEvent)) {
      // The current _mask cannot be used with _src1 so we release it.
      _srcEvent.release();
      return;
    }
    updateObject();
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}

void AccumulateWeighted::doUpdate() {

  cv::Mat sample;
  switch (_srcEvent.mat().depth()) {
    case CV_8U:
    case CV_8S:
    case CV_32F:
      sample = _srcEvent.mat();
      break;
    default:
      _srcEvent.payload().convertTo(sample,CV_32F);
      break;
  }


  if (_accumulator.empty() ||
      _srcEvent.payload().channels() != _accumulator.channels() ||
      _srcEvent.payload().size() != _accumulator.size()) {

    int alpha = 0;
    switch (_srcEvent.payload().depth()) {
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
    int type = CV_MAKETYPE(alpha,_srcEvent.payload().channels());
    _accumulator = cv::Mat(_srcEvent.payload().size(), type, cv::Scalar(0.0));
  }

  cv::Mat output;
  if (_maskEvent) {
    cv::accumulateWeighted(sample, _accumulator, _alpha, _maskEvent.mat() > 0);
  }
  else {
    cv::accumulateWeighted(sample, _accumulator, _alpha);
  }
  _accumulator.convertTo(output,_srcEvent.mat().type());
  emit dst(QVariant::fromValue(MatEvent(output,MatEvent::maxTimestamp(_srcEvent, _maskEvent))));
}
