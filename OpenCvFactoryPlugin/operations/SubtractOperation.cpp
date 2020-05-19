#include "operations/SubtractOperation.h"

#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(SubtractOperation)

SubtractOperation::SubtractOperation(QObject* parent)
  : QObject(parent)
  , _type(-1)
{
}

void SubtractOperation::outputType(int type) {
  _type = type;
  update();
}

void SubtractOperation::alpha(const cv::Mat &alpha) {
  _alphaMat = alpha;
  if (_maskMat.empty()) {
    _maskMat = cv::Mat(cv::Size(_alphaMat.cols, _alphaMat.rows), CV_8U, cv::Scalar(255));
  }
  if (_betaMat.empty()) {
    _betaMat = cv::Mat(cv::Size(_alphaMat.cols, _alphaMat.rows), _alphaMat.type(), cv::Scalar(0));
  }
  update();
}

void SubtractOperation::beta(const cv::Mat &beta) {
  _betaMat = beta;
  if (_maskMat.empty()) {
    _maskMat = cv::Mat(cv::Size(_betaMat.cols, _betaMat.rows), CV_8U, cv::Scalar(255));
  }
  if (_alphaMat.empty()) {
    _alphaMat = cv::Mat(cv::Size(_betaMat.cols, _betaMat.rows), _betaMat.type(), cv::Scalar(0));
  }
  update();
}

void SubtractOperation::mask(const cv::Mat &mask) {
  _maskMat = mask;
  update();
}

void SubtractOperation::update() {
  if (_alphaMat.cols == _betaMat.cols &&
      _alphaMat.rows == _betaMat.rows &&
      (!_maskMat.empty() && _alphaMat.cols == _maskMat.cols && _alphaMat.rows == _maskMat.rows)) {
    cv::Mat dst;
    cv::subtract(_alphaMat, _betaMat, dst, _maskMat, CV_32F);
    emit output(dst);
  }
}
