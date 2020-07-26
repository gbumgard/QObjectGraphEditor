#include "ApplyMask.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ApplyMask)

ApplyMask::ApplyMask(QObject* parent)
  : AbstractOpenCvObject(parent)
{
}

void ApplyMask::src(const MatEvent &srcEvent) {
  _srcEvent = srcEvent;
  // Check to see if _src1 and _src2 have the same attributes and are synchronized
  if (!_srcEvent.isSynchronizedMatch(_maskEvent)) {
    // The current mask cannot be used with src so we release it.
    _maskEvent.release();
    return;
  }
  update();
}

void ApplyMask::mask(const MatEvent &maskEvent) {
  _maskEvent = maskEvent;
  if (!_maskEvent.isSynchronizedMask(_srcEvent)) {
    // The current src cannot be used with mask so we release it.
    _srcEvent.release();
    return;
  }
  update();
}

void ApplyMask::update() {
  cv::Mat dstMat;
  _srcEvent.mat().copyTo(dstMat,_maskEvent.mat() > 0);
  emit dst(MatEvent(dstMat,MatEvent::maxTimestamp(_srcEvent,_maskEvent)));
}
