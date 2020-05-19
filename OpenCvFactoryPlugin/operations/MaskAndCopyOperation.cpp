#include "MaskAndCopyOperation.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(MaskAndCopyOperation)

MaskAndCopyOperation::MaskAndCopyOperation(QObject* parent)
  : QObject(parent)
{
}

void MaskAndCopyOperation::in(const cv::Mat &mat) {
  _input = mat;
  if (_mask.empty()) {
    _mask = cv::Mat(cv::Size(_input.cols, _input.rows), CV_8U, cv::Scalar(255));
  }
  update();
}

void MaskAndCopyOperation::mask(const cv::Mat &mat) {
  _mask = mat;
//  update();
}

void MaskAndCopyOperation::update() {
  if (!_input.empty() && !_mask.empty() &&
      _input.cols == _mask.cols &&
      _input.rows == _mask.rows) {
    if (_input.cols != _output.cols || _input.rows != _output.rows) {
      _output = cv::Mat(cv::Size(_input.cols, _input.rows), _input.type(), cv::Scalar(0));
    }
    _input.copyTo(_output,_mask);
    emit out(_output);
    emit maskUsed(_mask);
  }
}
