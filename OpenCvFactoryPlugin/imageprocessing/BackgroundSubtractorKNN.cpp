#include "BackgroundSubtractorKNN.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(BackgroundSubtractorKNN)

BackgroundSubtractorKNN::BackgroundSubtractorKNN(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _filterWeight(1.0)
    , _bkgndSubtractor(cv::createBackgroundSubtractorKNN())
{
}

void BackgroundSubtractorKNN::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat input;
        switch (matEvent.mat().depth()) {
        case CV_8U:
        case CV_8S:
            input = matEvent.mat();
            break;
        default:
            matEvent.mat().convertTo(input,CV_8U);
            break;
        }

        cv::Mat fgmask;
        _bkgndSubtractor->apply(input,fgmask);
        cv::Mat masked;
        input.copyTo(masked,fgmask);
        emit out(QVariant::fromValue(MatEvent(masked,matEvent.timestamp())));
        emit mask(QVariant::fromValue(MatEvent(fgmask,matEvent.timestamp())));
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}
