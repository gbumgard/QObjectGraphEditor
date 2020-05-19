#include "GaussianBlurFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(GaussianBlurFilter)

GaussianBlurFilter::GaussianBlurFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _kernelSizeX(3)
  , _kernelSizeY(3)
  , _sigmaX(0.0)
  , _sigmaY(0.0)
  , _borderType(BORDER_DEFAULT)
{
}

void GaussianBlurFilter::setKernelSizeX(int kernelSizeX) {
  if (kernelSizeX < 1) kernelSizeX = 3;
  _kernelSizeX = 1 + (kernelSizeX / 2) * 2;
  update();
}

void GaussianBlurFilter::setKernelSizeY(int kernelSizeY) {
  if (kernelSizeY < 1) kernelSizeY = 3;
  _kernelSizeY = 1 + (kernelSizeY / 2) * 2;
  update();
}

void GaussianBlurFilter::setSigmaX(double sigmaX) {
  _sigmaX = sigmaX;
  update();
}

void GaussianBlurFilter::setSigmaY(double sigmaY) {
  _sigmaY = sigmaY;
  update();
}

void GaussianBlurFilter::setBorderType(BorderType borderType) {
  _borderType = borderType;
  update();
}

void GaussianBlurFilter::in(const cv::Mat &mat) {
  _input = mat;
  update();
}

void GaussianBlurFilter::update() {
  if (!_input.empty()) {
    cv::Mat dst;
    if (_kernelSizeX >= 3 || _kernelSizeY >= 3) {
      cv::GaussianBlur(_input, dst, cv::Size(_kernelSizeX,_kernelSizeY), _sigmaX, _sigmaY, (cv::BorderTypes)_borderType);
    }
    else {
      _input.copyTo(dst);
    }
    emit out(dst);
  }
}
