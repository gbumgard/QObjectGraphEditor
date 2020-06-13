#include "ClipAndNormalize.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(ClipAndNormalize)

ClipAndNormalize::ClipAndNormalize(QObject* parent)
  : QObject(parent)
  , _minimum(0)
  , _maximum(255)
  , _range(255)
  , _offset(0)
{
}

void ClipAndNormalize::minimum(double value) {
  _minimum = value;
}

void ClipAndNormalize::maximum(double value) {
  _maximum = value;
}

void ClipAndNormalize::range(double value) {
  _range = value;
}

void ClipAndNormalize::offset(double value) {
  _offset = value;
}

void ClipAndNormalize::in(const TaggedMat &taggedMat) {
  double inputRange = _maximum - _minimum;
  double scale = _range / inputRange;
  cv::Mat dst = taggedMat.first.clone();
  dst -= _minimum;
  dst *= scale;
  dst += _offset;
  emit out(TaggedMat(dst,taggedMat.second));
}
