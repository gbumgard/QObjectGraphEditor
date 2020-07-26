#include "operations/AbsDifference.h"

#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AbsDifference)

AbsDifference::AbsDifference(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _src1Event(cv::Mat(),MatEvent::UNTIMED)
  , _src2Event(cv::Mat(),MatEvent::UNTIMED)
{
}

void AbsDifference::src1(const MatEvent &src1Event) {
  _src1Event = src1Event;
  // Check to see if _src1 and _src2 have the same attributes and are synchronized
  if (!_src1Event.isSynchronizedMatch(_src2Event)) {
    // The current _src2 cannot be used with _src1 so we release it.
    _src2Event.release();
    return;
  }
  update();
}

void AbsDifference::src2(const MatEvent &src2Event) {
  _src2Event = src2Event;
  // Check to see if _src1 and _src2 have the same attributes and are synchronized
  if (!_src2Event.isSynchronizedMatch(_src1Event)) {
    // The current _src1 cannot be used with _src2 so we release it.
    _src1Event.release();
    return;
  }
  update();
}

void AbsDifference::doUpdate() {
  cv::Mat dst;
  cv::absdiff(_src1Event.mat(), _src2Event.mat(), dst);
  emit AbsDifference::dst(MatEvent(dst,MatEvent::maxTimestamp(_src1Event, _src2Event)));
}
