#include "filters/MedianBlurFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>
#include <QDebug>

REGISTER_CLASS(MedianBlurFilter)

MedianBlurFilter::MedianBlurFilter(QObject* parent)
  : QObject(parent)
  , _aperatureSize(Aperature3x3)
{
}

void MedianBlurFilter::setAperatureSize(AperatureSize aperatureSize) {
  _aperatureSize = aperatureSize;
}

void MedianBlurFilter::in(const MatEvent &input) {
  if (!input.mat().empty()) {

    AperatureSize aperatureSize = _aperatureSize;
    if ((int)aperatureSize > 5 && input.mat().depth() != CV_8U) {
      qDebug() << "aperature size is too large for image type - limited to 5x5";
      aperatureSize = Aperature5x5;
    }

    cv::Mat src;
    switch (input.mat().depth()) {
    case CV_8U:
    case CV_16U:
    case CV_32F:
      src = input.mat();
      break;
    default:
      input.mat().convertTo(src,CV_32F);
      break;
    }

    try {
      cv::Mat dst;
      cv::medianBlur(src,dst,aperatureSize);
      cv::Mat output;
      dst.convertTo(output,input.mat().type());
      emit out(MatEvent(output,input.timestamp()));
    }
    catch(cv::Exception &e) {
      qDebug() << e.what();
    }
  }

}
