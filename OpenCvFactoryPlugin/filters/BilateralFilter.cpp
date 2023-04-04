#include "filters/BilateralFilter.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>
#include <QDebug>

REGISTER_CLASS(BilateralFilter)

BilateralFilter::BilateralFilter(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _srcEvent()
  , _filterSize(FilterSize_1)
  , _sigmaColor(10)
  , _sigmaSpace(10)
  , _borderType(DEFAULT)
{
}

void BilateralFilter::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);
      if (!matEvent.mat().empty()) {
          cv::Mat src(matEvent.mat());
          switch (matEvent.mat().depth()) {
          case CV_8U:
          case CV_32F:
              break;
          default:
              matEvent.mat().convertTo(src,CV_32F);
              break;
          }

          try {
              cv::Mat output;
              cv::bilateralFilter(src, output, _filterSize != SigmaSpace ? (int)_filterSize : -1, _sigmaColor, _sigmaSpace, _borderType);
              emit out(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
          }
          catch(cv::Exception &e) {
              qDebug() << e.what();
          }
      }
  }
  else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}

#if 0
void BilateralFilter::update() {

  cv::Mat currentInput;
  FilterSize currentFilterSize;
  qreal currentSigmaColor;
  qreal currentSigmaSpace;
  BorderType currentBorderType;
  {
    WaitForUpdateLock lock(this);
    if (_stop) return;
    currentInput = _input.clone();
    _input = cv::Mat();
    if (!currentInput.empty()) {
      currentFilterSize = _filterSize;
      currentSigmaColor = _sigmaColor;
      currentSigmaSpace = _sigmaSpace;
      currentBorderType = _borderType;
    }
    else {
      return;
    }
  }

  if (!currentInput.empty()) {
    cv::Mat src(currentInput);
    switch (currentInput.depth()) {
    case CV_8U:
    case CV_32F:
      break;
    default:
      currentInput.convertTo(src,CV_32F);
      break;
    }

    try {
      cv::Mat dst;
      cv::bilateralFilter(src, dst, currentFilterSize != SigmaSpace ? (int)currentFilterSize : -1, currentSigmaColor, currentSigmaSpace, currentBorderType);
      emit out(dst);
    }
    catch(cv::Exception &e) {
      qDebug() << e.what();
    }
  }

}
#endif
