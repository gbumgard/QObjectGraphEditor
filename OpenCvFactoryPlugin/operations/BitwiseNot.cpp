#include "BitwiseNot.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(BitwiseNot)

BitwiseNot::BitwiseNot(QObject* parent)
  : AbstractOpenCvObject(parent)
{
}

void BitwiseNot::src(const MatEvent &srcEvent) {
  _srcEvent = srcEvent;
  if (!_srcEvent.isSynchronizedMask(_maskEvent)) {
    // The current mask cannot be used with src so we release it.
    _maskEvent.release();
    return;
  }
  update();
}

void BitwiseNot::mask(const MatEvent &maskEvent) {
  _maskEvent = maskEvent;
  if (!_maskEvent.isSynchronizedMask(_srcEvent)) {
    // The current src cannot be used with mask so we release it.
    _srcEvent.release();
    return;
  }
  update();
}

void BitwiseNot::update() {
  cv::Mat dstMat;
  if (_maskEvent) {
    cv::bitwise_not(_srcEvent.mat(),dstMat,_maskEvent.mat() > 0);
  }
  else {
    cv::bitwise_not(_srcEvent.mat(),dstMat);
  }
  emit dst(MatEvent(dstMat,MatEvent::maxTimestamp(_srcEvent,_maskEvent)));
}
