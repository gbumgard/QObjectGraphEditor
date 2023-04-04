#include "AddWeightedOperation.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AddWeightedOperation)

AddWeightedOperation::AddWeightedOperation(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _alphaWeight(0.5)
  , _betaWeight(0.5)
  , _gammaOffset(0.0)
{
}

void AddWeightedOperation::alphaWeight(double alphaWeight) {
  _alphaWeight = alphaWeight;
  update();
}

void AddWeightedOperation::betaWeight(double betaWeight) {
  _betaWeight = betaWeight;
  update();
}

void AddWeightedOperation::gammaOffset(double gammaOffset) {
  _gammaOffset = gammaOffset;
  update();
}

void AddWeightedOperation::alpha(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);
    _alphaMat = matEvent.mat();
    _timestamp = matEvent.timestamp();
    if (_betaMat.empty()) {
      _betaMat = cv::Mat(_alphaMat.size(),_alphaMat.type(),cv::Scalar(0));
    }
    update();
  }
}

void AddWeightedOperation::beta(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);
    _betaMat = matEvent.mat();
    _timestamp = matEvent.timestamp();
    if (_betaMat.empty()) {
      _alphaMat = cv::Mat(_betaMat.size(),_betaMat.type(),cv::Scalar(0));
    }
    update();
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}

void AddWeightedOperation::update() {
  if (_alphaMat.cols == _betaMat.cols &&
      _alphaMat.rows == _betaMat.rows &&
      _alphaMat.size() == _betaMat.size() &&
      _alphaMat.channels() == _betaMat.channels()) {
    cv::Mat output(cv::Size(_alphaMat.cols, _alphaMat.rows), _alphaMat.type());
    cv::addWeighted(_alphaMat, _alphaWeight, _betaMat, _betaWeight, _gammaOffset, output, _alphaMat.type());
    emit dst(QVariant::fromValue(MatEvent(output,_timestamp)));
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
