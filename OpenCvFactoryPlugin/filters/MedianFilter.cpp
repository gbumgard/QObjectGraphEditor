#include "filters/MedianFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(MedianFilter)

MedianFilter::MedianFilter(QObject* parent)
  : QObject(parent)
  , _aperatureSize(3)
{
}

void MedianFilter::setAperatureSize(int aperatureSize) {
  if (aperatureSize < 3) aperatureSize = 3;
  _aperatureSize = 1 + (aperatureSize / 2) * 2;
  update();
}

void MedianFilter::in(const cv::Mat &mat) {
  _input = mat;
  update();
}

void MedianFilter::update() {
  if (!_input.empty()) {
    int depth = _input.depth();
    if  (_aperatureSize > 5) {
      if (depth != CV_8U) _aperatureSize = 5;
    }
    cv::Mat dst;
    if (_aperatureSize >= 3) {
      cv::medianBlur(_input,dst,_aperatureSize);
    }
    else {
      _input.copyTo(dst);
    }
    emit out(dst);
  }
}
