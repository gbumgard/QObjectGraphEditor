#include "InRangeFilter.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(InRangeFilter)

InRangeFilter::InRangeFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _minimum(std::numeric_limits<double>::lowest())
  , _maximum(std::numeric_limits<double>::max())
  //, _low(std::numeric_limits<double>::lowest())
  //, _high(std::numeric_limits<double>::max())
{
}

void InRangeFilter::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);
    cv::Mat aboveLowerMask = matEvent.mat() >= _minimum;
    cv::Mat belowUpperMask = matEvent.mat() <= _maximum;
    cv::Mat inRangeMask = aboveLowerMask & belowUpperMask;
    cv::Mat output;
    matEvent.mat().copyTo(output,inRangeMask);
    //output.setTo(cv::Scalar(_low),~aboveLowerMask);
    //output.setTo(cv::Scalar(_high),~belowUpperMask);
    emit out(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
