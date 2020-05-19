#include "MaskOperation.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(MaskOperation)

MaskOperation::MaskOperation(QObject* parent)
  : AbstractOpenCvObject(parent)
{
}

void MaskOperation::in(const cv::Mat &mat) {
  _input = mat;
  if (_mask.empty()) {
    _mask = cv::Mat(cv::Size(_input.cols, _input.rows), CV_8U, cv::Scalar(255));
  }
  update();
}

void MaskOperation::mask(const cv::Mat &mat) {
  _mask = mat;
  update();
}

void MaskOperation::update() {
  if (!_input.empty() && !_mask.empty() &&
      _input.cols == _mask.cols &&
      _input.rows == _mask.rows) {
    cv::Mat dst(cv::Size(_input.cols, _input.rows), _input.type(), cv::Scalar(0));
    _input.copyTo(dst,_mask);
    emit out(dst);
    emit maskUsed(_mask);
  }
}
