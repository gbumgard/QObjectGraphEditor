#include "MorphologyTransformation.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(MorphologyTransformation)

MorphologyTransformation::MorphologyTransformation(QObject* parent)
  : QObject(parent)
  , _operation(MorphologyTransformation::OPEN)
  , _shape(MorphologyTransformation::ELLIPSE)
  , _kernelSize(KERNEL_3X3)
  , _iterations(1)
{
}

void MorphologyTransformation::operation(Operation operation) {
  _operation = operation;
}

void MorphologyTransformation::shape(Shape shape) {
  _shape = shape;
}

void MorphologyTransformation::kernelSize(KernelSize kernelSize) {
  _kernelSize = kernelSize;
}

void MorphologyTransformation::iterations(int iterations) {
  _iterations = iterations >= 0 ? iterations : 0;
}

void MorphologyTransformation::in(const MatEvent& input) {
  if (!input.mat().empty()) {
    int kernelSize = (int)_kernelSize;
    cv::Mat structuringElement = cv::getStructuringElement(_shape, cv::Size(kernelSize,kernelSize));
    cv::Mat output;
    if (_iterations > 0) {
      cv::morphologyEx(input.mat(),output,_operation,structuringElement,cv::Point(-1,-1),_iterations);
    }
    else  {
      output = input.mat();
    }
    emit out(MatEvent(output,input.timestamp()));
  }
}
