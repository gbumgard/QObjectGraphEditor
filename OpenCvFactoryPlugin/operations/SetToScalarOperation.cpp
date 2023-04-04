#include "SetToScalarOperation.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(SetToScalarOperation)

SetToScalarOperation::SetToScalarOperation(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _value(0)
{
}

void SetToScalarOperation::in(const cv::Mat &inputMat) {
  _input = inputMat;
  update();
}

void SetToScalarOperation::mask(const cv::Mat &mask) {
  _mask = mask;
  update();
}
void SetToScalarOperation::value(double value) {
  _value = value;
  update();
}

void SetToScalarOperation::update() {
  if ((!_mask.empty() && _input.cols == _mask.cols && _input.rows == _mask.rows)) {
    cv::Mat dst;
    _input.copyTo(dst);
    dst.setTo(cv::Scalar(_value), _mask);
    emit out(dst);
  }
}
