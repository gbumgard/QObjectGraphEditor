#include "AnisotropicDiffusion.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

REGISTER_CLASS(AnisotropicDiffusion)

AnisotropicDiffusion::AnisotropicDiffusion(QObject* parent)
  : QObject(parent)
  , _alpha(.5)
  , _sensitivity(1)
  , _iterations(1)
{
}

void AnisotropicDiffusion::alpha(double alpha) {
  if (alpha > 0.0 && alpha <= 1.0) {
    _alpha = alpha;
    update();
  }
}

void AnisotropicDiffusion::sensitivity(double sensitivity) {
  if (sensitivity != 0 && sensitivity != _sensitivity) {
    _sensitivity = sensitivity;
    update();
  }
}

void AnisotropicDiffusion::iterations(int iterations) {
  if (iterations > 0 && iterations != _iterations) {
    _iterations = iterations;
    update();
  }
}

void AnisotropicDiffusion::in(const cv::Mat &mat) {
  if (mat.type() != CV_8UC3) {
    cv::Mat cv8UC3;
    cv::cvtColor(mat,cv8UC3,cv::COLOR_GRAY2RGB);
    _input = cv8UC3;
  }
  else {
    _input = mat;
  }
  update();
}

void AnisotropicDiffusion::update() {
  if (!_input.empty()) {
    cv::Mat dst(cv::Size(_input.cols, _input.rows), _input.type(), cv::Scalar(0));
    cv::ximgproc::anisotropicDiffusion(_input,dst,_alpha,_sensitivity,_iterations);
    emit out(dst);
  }
}
