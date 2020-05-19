#include "MorphologyTransformation.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(MorphologyTransformation)

MorphologyTransformation::MorphologyTransformation(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _operation(MorphologyTransformation::OPEN)
  , _shape(MorphologyTransformation::ELLIPSE)
  , _kernelSize(KERNEL_3X3)
  , _iterations(1)
{
}

void MorphologyTransformation::operation(Operation operation) {
  _operation = operation;
  update();
}

void MorphologyTransformation::shape(Shape shape) {
  _shape = shape;
  update();
}

void MorphologyTransformation::kernelSize(KernelSize kernelSize) {
  _kernelSize = kernelSize;
  update();
}

void MorphologyTransformation::iterations(int iterations) {
  _iterations = iterations >= 0 ? iterations : 0;
  update();
}

void MorphologyTransformation::in(const cv::Mat& mat) {
  _input = mat;
  update();
}

void MorphologyTransformation::update() {
  if (!_input.empty()) {
    int kernelSize = (int)_kernelSize;
    cv::Mat structuringElement = cv::getStructuringElement(_shape, cv::Size(kernelSize,kernelSize));
    cv::Mat dst;
    if (_iterations > 0) {
      cv::morphologyEx(_input,dst,_operation,structuringElement,cv::Point(-1,-1),_iterations);
    }
    else  {
      _input.copyTo(dst);
    }
    emit out(dst);
  }
}
