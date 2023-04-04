#include "BitwiseOr.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(BitwiseOr)

BitwiseOr::BitwiseOr(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _src1Event(cv::Mat(),MatEvent::UNTIMED)
  , _src2Event(cv::Mat(),MatEvent::UNTIMED)
  , _maskEvent(cv::Mat(),MatEvent::UNTIMED)
{
}

void BitwiseOr::src1(const MatEvent &src1Event) {
  _src1Event = src1Event;
  if (!_src1Event.isSynchronizedMatch(_src2Event)) {
    // The current src1 cannot be used with src2 so we release it.
    _src2Event.release();
    return;
  }
  if (!_src1Event.isSynchronizedMask(_maskEvent)) {
    // The current mask cannot be used with src1 so we release it.
    _maskEvent.release();
    return;
  }
  update();
}

void BitwiseOr::src2(const MatEvent &src2Event) {
  _src2Event = src2Event;
  if (!_src2Event.isSynchronizedMatch(_src1Event)) {
    // The current src2 cannot be used with src1 so we release it.
    _src1Event.release();
    return;
  }
  if (!_src2Event.isSynchronizedMask(_maskEvent)) {
    // The current mask cannot be used with src2 so we release it.
    _maskEvent.release();
    return;
  }
  update();
}

void BitwiseOr::mask(const MatEvent &maskEvent) {
  _maskEvent = maskEvent;
  if (!_maskEvent.isSynchronizedMask(_src1Event)) {
    // The current src1 cannot be used with mask so we release it.
    _src1Event.release();
    return;
  }
  if (!_maskEvent.isSynchronizedMask(_src2Event)) {
    // The current src2 cannot be used with mask so we release it.
    _src2Event.release();
    return;
  }
  update();
}

void BitwiseOr::doUpdate() {
  cv::Mat dstMat;
  if (_maskEvent) {
    cv::bitwise_or(_src1Event.mat(),_src2Event.mat(),dstMat,_maskEvent.mat());
  }
  else {
    cv::bitwise_or(_src1Event.mat(),_src2Event.mat(),dstMat);
  }
  emit dst(MatEvent(dstMat,MatEvent::maxTimestamp(_src1Event,_src2Event,_maskEvent)));
}
