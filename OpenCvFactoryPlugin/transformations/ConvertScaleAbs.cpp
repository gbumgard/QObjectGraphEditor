#include "ConvertScaleAbs.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ConvertScaleAbs)

ConvertScaleAbs::ConvertScaleAbs(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _scale(1.0)
  , _offset(0.0)
{
}

void ConvertScaleAbs::scale(double scale) {
  _scale = scale;
}

void ConvertScaleAbs::offset(double offset) {
  _offset = offset;
}

void ConvertScaleAbs::src(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);
    cv::Mat output  ;
    cv::convertScaleAbs(matEvent.mat(), output, _scale, _offset);
    emit dst(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
