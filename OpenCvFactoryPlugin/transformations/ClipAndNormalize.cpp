#include "ClipAndNormalize.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(ClipAndNormalize)

ClipAndNormalize::ClipAndNormalize(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _minimum(0)
  , _maximum(255)
  , _range(255)
  , _offset(0)
{
}

void ClipAndNormalize::minimum(double value) {
  _minimum = value;
  update();
}

void ClipAndNormalize::maximum(double value) {
  _maximum = value;
  update();
}

void ClipAndNormalize::range(double value) {
  _range = value;
  update();
}

void ClipAndNormalize::offset(double value) {
  _offset = value;
  update();
}

void ClipAndNormalize::in(const cv::Mat &mat) {
  _input = mat;
  update();
}

void ClipAndNormalize::update() {

  static const QMetaMethod maxValueMethod = QMetaMethod::fromSignal(&ClipAndNormalize::maxValueIn);
  static const QMetaMethod minValueMethod = QMetaMethod::fromSignal(&ClipAndNormalize::minValueIn);

  cv::Mat mat = _input.clone();
  if (isSignalConnected(maxValueMethod) || isSignalConnected(minValueMethod)) {
    double min, max;
    cv::minMaxLoc(mat,&min,&max);
    emit minValueIn(min);
    emit maxValueIn(max);
  }

  double inputRange = _maximum - _minimum;
  double scale = _range / inputRange;
  mat -= _minimum;
  mat *= scale;
  mat += _offset;
  emit out(mat);
}
