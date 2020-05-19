#include "ConvertScaleAbs.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ConvertScaleAbs)

ConvertScaleAbs::ConvertScaleAbs(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _scale(1.0)
  , _offset(0.0)
{
}

void ConvertScaleAbs::scale(double scale) {
  _scale = scale;
  update();
}

void ConvertScaleAbs::offset(double offset) {
  _offset = offset;
  update();
}

void ConvertScaleAbs::in(const cv::Mat &src) {
  _input = src;
  update();
}

void ConvertScaleAbs::update() {
  if (!_input.empty()) {
    cv::Mat dst;
    cv::convertScaleAbs(_input, dst, _scale, _offset);
    emit out(dst);
  }
}
