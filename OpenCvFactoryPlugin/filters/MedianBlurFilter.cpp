#include "filters/MedianBlurFilter.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>
#include <QDebug>

REGISTER_CLASS(MedianBlurFilter)

MedianBlurFilter::MedianBlurFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _aperatureSize(Aperature3x3)
{
}

void MedianBlurFilter::setAperatureSize(AperatureSize aperatureSize) {
  _aperatureSize = aperatureSize;
}

void MedianBlurFilter::src(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);
    AperatureSize aperatureSize = _aperatureSize;
    if ((int)aperatureSize > 5 && matEvent.mat().depth() != CV_8U) {
      qDebug() << "aperature size is too large for image type - limited to 5x5";
      aperatureSize = Aperature5x5;
    }

    cv::Mat src;
    switch (matEvent.mat().depth()) {
    case CV_8U:
    case CV_16U:
    case CV_32F:
      src = matEvent.mat();
      break;
    default:
      matEvent.mat().convertTo(src,CV_32F);
      break;
    }

    try {
      cv::Mat blurred;
      cv::medianBlur(src,blurred,aperatureSize);
      cv::Mat output;
      blurred.convertTo(output,matEvent.mat().type());
      emit dst(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
    }
    catch(cv::Exception &e) {
      qDebug() << e.what();
    }
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
