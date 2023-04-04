#include "BitwiseAnd.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(BitwiseAnd)

BitwiseAnd::BitwiseAnd(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _src1Event(cv::Mat(),MatEvent::UNTIMED)
  , _src2Event(cv::Mat(),MatEvent::UNTIMED)
  , _maskEvent(cv::Mat(),MatEvent::UNTIMED)
{
}

void BitwiseAnd::src1(const MatEvent &event) {
  _src1Event = event;
  // Check to see if _src1 and _src2 have the same attributes and are synchronized
  if (!_src1Event.isSynchronizedMatch(_src2Event)) {
    // The current _src2 cannot be used with _src1 so we release it.
    _src2Event.release();
    return;
  }
  if (!_src1Event.isSynchronizedMask(_maskEvent)) {
    // The current _mask cannot be used with _src1 so we release it.
    _maskEvent.release();
    return;
  }
  update();
}

void BitwiseAnd::src2(const MatEvent &src2Event) {
  _src2Event = src2Event;
  // Check to see if _src1 and _src2 have the same attributes and are synchronized
  if (!_src2Event.isSynchronizedMatch(_src1Event)) {
    // The current _src1 cannot be used with _src2 so we release it.
    _src1Event.release();
    return;
  }
  if (!_src2Event.isSynchronizedMask(_maskEvent)) {
    // The current _mask cannot be used with _src2 so we release it.
    _maskEvent.release();
    return;
  }
  update();
}

void BitwiseAnd::mask(const MatEvent &maskEvent) {
  _maskEvent = maskEvent;
  if (!_maskEvent.isSynchronizedMask(_src1Event)) {
    // The current _src1 cannot be used with _mask so we release it.
    _src1Event.release();
    return;
  }
  if (!_maskEvent.isSynchronizedMask(_src2Event)) {
    // The current _src2 cannot be used with _mask so we release it.
    _src2Event.release();
    return;
  }
  update();
}

void BitwiseAnd::doUpdate() {
  cv::Mat dstMat;
  if (_maskEvent) {
    cv::bitwise_and(_src1Event.mat(), _src1Event.mat(), dstMat, _maskEvent.mat() > 0);
  }
  else {
    cv::bitwise_and(_src1Event.mat(), _src1Event.mat(), dstMat, cv::noArray());
  }
  emit dst(MatEvent(dstMat,MatEvent::maxTimestamp(_src1Event, _src2Event, _maskEvent)));
}
