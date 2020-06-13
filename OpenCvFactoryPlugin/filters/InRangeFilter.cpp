#include "InRangeFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(InRangeFilter)

InRangeFilter::InRangeFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _minimum(std::numeric_limits<double>::lowest())
  , _maximum(std::numeric_limits<double>::max())
{
}

void InRangeFilter::in(const TaggedMat& taggedMat) {

  if (!taggedMat.first.empty()) {
    cv::Mat aboveLowerMask = taggedMat.first >= _minimum;
    cv::Mat belowUpperMask = taggedMat.first <= _maximum;
    cv::Mat inRangeMask = aboveLowerMask & belowUpperMask;
    cv::Mat dst;
    taggedMat.first.copyTo(dst,inRangeMask);
    dst.setTo(cv::Scalar(_low),~aboveLowerMask);
    dst.setTo(cv::Scalar(_high),~belowUpperMask);
    emit out(TaggedMat(dst,taggedMat.second));
  }
}
