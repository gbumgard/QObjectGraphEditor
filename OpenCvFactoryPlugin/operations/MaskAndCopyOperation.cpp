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
        _input = matEvent.mat();
        if (_mask.empty()) {
            _mask = cv::Mat(cv::Size(_input.cols, _input.rows), CV_8U, cv::Scalar(255));
        }
        _input.copyTo(_output,_mask);
        emit out(QVariant::fromValue(MatEvent(_output,matEvent.timestamp())));
        emit maskUsed(QVariant::fromValue(MatEvent(_mask,matEvent.timestamp())));
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

void MaskAndCopyOperation::mask(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        _mask = matEvent.mat();
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

