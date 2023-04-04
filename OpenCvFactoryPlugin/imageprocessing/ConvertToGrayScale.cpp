#include "ConvertToGrayScale.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(ConvertToGrayScale)

ConvertToGrayScale::ConvertToGrayScale(QObject* parent)
  : QObject(parent)
{
}

void ConvertToGrayScale::in(const cv::Mat &mat) {
  _input = mat;
  update();
}

void ConvertToGrayScale::update() {
  cv::Mat dst;
  cv::cvtColor(_input,dst,cv::COLOR_BGR2GRAY);
  emit out(dst);
}
