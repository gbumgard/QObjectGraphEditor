#include "RunningAverageFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(RunningAverageFilter)

RunningAverageFilter::RunningAverageFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _weight(1.0)
{
}

void RunningAverageFilter::weight(double weight) {
  if (weight <= 1.0 && weight >= 0.0) _weight = weight;
  update();
}

void RunningAverageFilter::in(const cv::Mat &mat) {
  _input = mat;
  update();
}

void RunningAverageFilter::update() {
  if (!_input.empty()) {
    if (_average.empty()
        || _input.rows != _average.rows
        || _input.cols != _average.cols
        || _input.channels() != _average.channels()) {

      int depth;
      switch (_input.depth()) {
        case CV_8U:
        case CV_8S:
        case CV_16U:
        case CV_16S:
          depth = CV_32F;
          break;
        case CV_32S:
        case CV_32F:
          depth = CV_64F;
          break;
        default:
          break;
      }

      int type = CV_MAKETYPE(depth,_input.channels());
      _average = cv::Mat(cv::Size(_input.cols, _input.rows), type, cv::Scalar(0.0));
    }
    cv::accumulateWeighted(_input, _average, _weight);
    emit out(_average);
  }
}
