#include "BackgroundSubtractionMask.h"
#include "OpenCvFactoryPlugin.h"
#include <QDebug>

REGISTER_CLASS(BackgroundSubtractionMask)

BackgroundSubtractionMask::BackgroundSubtractionMask(QObject* parent)
  : QObject(parent)
  , _pSubtractor(cv::createBackgroundSubtractorKNN().dynamicCast<cv::BackgroundSubtractor>())
  , _algorithm(KNN)
  , _learningRate(1.0)
{
}

void BackgroundSubtractionMask::algorithm(Algorithm algorithm) {
  if (algorithm != _algorithm) {
    _algorithm = algorithm;
    switch (_algorithm) {
      case KNN:
        _pSubtractor = cv::createBackgroundSubtractorKNN().dynamicCast<cv::BackgroundSubtractor>();
        break;
      case MOG2:
      default:
        _pSubtractor = cv::createBackgroundSubtractorMOG2().dynamicCast<cv::BackgroundSubtractor>();
        break;
    }
  }
}

void BackgroundSubtractionMask::learningRate(double learningRate) {
  _learningRate = learningRate < 0.0 ? -1.0 : learningRate > 1.0 ? 1.0 : learningRate;
}

void BackgroundSubtractionMask::in(const cv::Mat &inputImage) {
  if (!inputImage.empty()) {
    cv::Mat maskImage(inputImage.size(),inputImage.type());
    _pSubtractor->apply(inputImage,maskImage,_learningRate);
    emit mask(maskImage);
    cv::Mat bgndImage;
    _pSubtractor->getBackgroundImage(bgndImage);
    emit bgnd(bgndImage);
  }
}
