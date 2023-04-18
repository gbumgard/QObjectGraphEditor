#include "MaskAndCopyOperation.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(MaskAndCopyOperation)

MaskAndCopyOperation::MaskAndCopyOperation(QObject* parent)
  : AbstractOpenCvObject(parent)
{
}

void MaskAndCopyOperation::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        _input = matEvent.mat().clone();
        if (_mask.empty() || _mask.size != _input.size) {
            _mask = cv::Mat(cv::Size(_input.cols, _input.rows), CV_8UC1, cv::Scalar(255));
        }
        if (_output.empty() || _output.size != _input.size || _output.type() != _input.type()) {
            _output = cv::Mat(cv::Size(_input.cols, _input.rows), _input.type(), cv::Scalar(0));
        }
        _input.copyTo(_output,_mask);
        emit out(QVariant::fromValue(MatEvent(_output,matEvent.timestamp())));
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

void MaskAndCopyOperation::mask(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        matEvent.mat().convertTo(_mask,CV_8UC1);
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

