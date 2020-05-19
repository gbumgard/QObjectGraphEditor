#include "AddOperation.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AddOperation)

AddOperation::AddOperation(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _type(-1)
{
}

void AddOperation::setOutputType(int type) {
  _type = type;
  update();
}

void AddOperation::alpha(const cv::Mat &alpha) {
  _alphaMat = alpha;
  if (_maskMat.empty()) {
    _maskMat = cv::Mat(cv::Size(_alphaMat.cols, _alphaMat.rows), CV_8U, cv::Scalar(255));
  }
  if (_betaMat.empty()) {
    _betaMat = cv::Mat(cv::Size(_alphaMat.cols, _alphaMat.rows), _alphaMat.type(), cv::Scalar(0));
  }
  update();
}

void AddOperation::beta(const cv::Mat &beta) {
  _betaMat = beta;
  if (_maskMat.empty()) {
    _maskMat = cv::Mat(cv::Size(_betaMat.cols, _betaMat.rows), CV_8U, cv::Scalar(255));
  }
  if (_alphaMat.empty()) {
    _alphaMat = cv::Mat(cv::Size(_betaMat.cols, _betaMat.rows), _betaMat.type(), cv::Scalar(0));
  }
  update();
}

void AddOperation::mask(const cv::Mat &mask) {
  _maskMat = mask;
  update();
}

void AddOperation::update() {
  cv::Mat alpha = _alphaMat;
  cv::Mat beta = _betaMat;
  cv::Mat mask = _maskMat;
  if (alpha.cols == beta.cols &&
      alpha.rows == beta.rows &&
      alpha.type() == beta.type() &&
      (!mask.empty() && alpha.cols == mask.cols && alpha.rows == mask.rows)) {
    int type = _type == -1 ? alpha.type() : _type;
    cv::Mat dst(cv::Size(alpha.cols, alpha.rows), type, cv::Scalar(0));
    cv::add(alpha, beta, dst, mask, _type);
    emit out(dst);
    emit maskUsed(_maskMat);
  }
}
