#include "InRangeFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(InRangeFilter)

InRangeFilter::InRangeFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _minimum(std::numeric_limits<double>::lowest())
  , _maximum(std::numeric_limits<double>::max())
  , _low(std::numeric_limits<double>::lowest())
  , _high(std::numeric_limits<double>::max())
{
}

void InRangeFilter::in(const MatEvent& event) {

  if (!event.payload().empty()) {
    cv::Mat aboveLowerMask = event.payload() >= _minimum;
    cv::Mat belowUpperMask = event.payload() <= _maximum;
    cv::Mat inRangeMask = aboveLowerMask & belowUpperMask;
    cv::Mat dst;
    event.payload().copyTo(dst,inRangeMask);
    dst.setTo(cv::Scalar(_low),~aboveLowerMask);
    dst.setTo(cv::Scalar(_high),~belowUpperMask);
    emit out(MatEvent(dst,event.timestamp()));
  }
}
