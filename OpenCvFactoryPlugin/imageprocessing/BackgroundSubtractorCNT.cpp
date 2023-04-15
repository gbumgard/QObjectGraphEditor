#include "BackgroundSubtractorCNT.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(BackgroundSubtractorCNT)

BackgroundSubtractorCNT::BackgroundSubtractorCNT(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _learningRate(1.0)
    , _bkgndSubtractor(cv::bgsegm::createBackgroundSubtractorCNT())
{
    _useHistory = _bkgndSubtractor->getUseHistory();
    _minPixelStability = _bkgndSubtractor->getMinPixelStability();
    _maxPixelStability = _bkgndSubtractor->getMaxPixelStability();
}

void BackgroundSubtractorCNT::in(const QVariant &variant) {
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

        _bkgndSubtractor->setUseHistory(_useHistory);
        _bkgndSubtractor->setMinPixelStability(_minPixelStability);
        _bkgndSubtractor->setMaxPixelStability(_maxPixelStability);


        cv::Mat fgmask;
        _bkgndSubtractor->apply(input,fgmask,_learningRate);

        cv::Mat fore;
        input.copyTo(fore,fgmask);
        emit inFore(QVariant::fromValue(MatEvent(fore,matEvent.timestamp())));
        cv::Mat back;
        input.copyTo(back,~fgmask);
        emit inBack(QVariant::fromValue(MatEvent(back,matEvent.timestamp())));
        emit mask(QVariant::fromValue(MatEvent(fgmask,matEvent.timestamp())));
        cv::Mat bkgndImage;
        _bkgndSubtractor->getBackgroundImage(bkgndImage);
        emit bkgnd(QVariant::fromValue(MatEvent(bkgndImage,matEvent.timestamp())));
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}
