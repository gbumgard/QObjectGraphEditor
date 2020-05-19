#include "operations/AbsDifferenceOperation.h"

#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AbsDifferenceOperations)

AbsDifferenceOperations::AbsDifferenceOperations(QObject* parent)
  : QObject(parent)
{
}


void AbsDifferenceOperations::alpha(const cv::Mat &alpha) {
  _alphaMat = alpha;
  if (_betaMat.empty()) {
    _betaMat = cv::Mat(cv::Size(_alphaMat.cols, _alphaMat.rows), _alphaMat.type(), cv::Scalar(0));
  }
  update();
}

void AbsDifferenceOperations::beta(const cv::Mat &beta) {
  _betaMat = beta;
  if (_alphaMat.empty()) {
    _alphaMat = cv::Mat(cv::Size(_betaMat.cols, _betaMat.rows), _betaMat.type(), cv::Scalar(0));
  }
  update();
}

void AbsDifferenceOperations::update() {
  cv::Mat alpha = _alphaMat;
  cv::Mat beta = _betaMat;
  if (alpha.cols == beta.cols &&
      alpha.rows == beta.rows &&
      alpha.type() == beta.type()) {
    cv::Mat dst(cv::Size(alpha.cols, alpha.rows), alpha.type(), cv::Scalar(0));
    cv::absdiff(alpha, beta, dst);
    emit out(dst);
  }
}
