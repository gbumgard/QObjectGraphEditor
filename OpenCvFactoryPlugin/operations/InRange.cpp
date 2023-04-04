#include "InRange.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(InRange)

InRange::InRange(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _srcEvent(cv::Mat(),MatEvent::UNTIMED)
  , _lowerbEvent(cv::Mat(),MatEvent::UNTIMED)
  , _upperbEvent(cv::Mat(),MatEvent::UNTIMED)
{
}

void InRange::src(const MatEvent &srcEvent) {
  _srcEvent = srcEvent;
  if (!srcEvent.isSynchronizedMatch(_upperbEvent)) {
    // The current src cannot be used with upperb so we release it.
    _upperbEvent.release();
    return;
  }
  if (!srcEvent.isSynchronizedMatch(_lowerbEvent)) {
    // The current mask cannot be used with lowerb so we release it.
    _lowerbEvent.release();
    return;
  }
  update();
}

void InRange::lowerb(const MatEvent &lowerbEvent) {
  _lowerbEvent = lowerbEvent;
  if (!_lowerbEvent.isSynchronizedMatch(_srcEvent)) {
    // The current src cannot be used with lowerb mat so we release it.
    _srcEvent.release();
    return;
  }
  if (!_lowerbEvent.isSynchronizedMatch(_upperbEvent)) {
    // The current src cannot be used with upperb mat so we release it.
    _upperbEvent.release();
    return;
  }
  update();
}

void InRange::upperb(const MatEvent &upperbEvent) {
  _upperbEvent = upperbEvent;
  if (!_upperbEvent.isSynchronizedMatch(_srcEvent)) {
    // The current src cannot be used with upperb so we release it.
    _srcEvent.release();
    return;
  }
  if (!_upperbEvent.isSynchronizedMatch(_lowerbEvent)) {
    // The current mask cannot be used with lowerb so we release it.
    _lowerbEvent.release();
    return;
  }
  update();
}

void InRange::doUpdate() {
  cv::Mat dstMat;
  cv::inRange(_srcEvent.mat(), _lowerbEvent.mat(), _upperbEvent.mat(), dstMat);
  emit dst(MatEvent(dstMat,MatEvent::maxTimestamp(_srcEvent,_lowerbEvent,_upperbEvent)));
}
