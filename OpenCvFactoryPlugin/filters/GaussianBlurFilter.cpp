#include "GaussianBlurFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(GaussianBlurFilter)

GaussianBlurFilter::GaussianBlurFilter(QObject* parent)
  : QObject(parent)
  , _kernelSizeX(3)
  , _kernelSizeY(3)
  , _sigmaX(0.0)
  , _sigmaY(0.0)
  , _borderType(BORDER_DEFAULT)
{
}

void GaussianBlurFilter::in(const MatEvent &input) {
  if (!input.mat().empty()) {
    cv::Mat output;
    if (_kernelSizeX >= 3 || _kernelSizeY >= 3) {
      cv::GaussianBlur(input.mat(), output, cv::Size(_kernelSizeX,_kernelSizeY), _sigmaX, _sigmaY, (cv::BorderTypes)_borderType);
    }
    else {
      input.mat().copyTo(output);
    }
    emit out(MatEvent(output,input.timestamp()));
  }
}
