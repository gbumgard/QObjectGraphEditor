#include "GaussianBlurFilter.h"
#include "OpenCvFactoryPlugin.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(GaussianBlurFilter)

GaussianBlurFilter::GaussianBlurFilter(QObject* parent)
  : QObject(parent)
  , _kernelSizeX(3)
  , _kernelSizeY(3)
  , _sigmaX(0.0)
  , _sigmaY(0.0)
  , _borderType(BORDER_DEFAULT)
{
}

void GaussianBlurFilter::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat output;
        if (_kernelSizeX >= 3 || _kernelSizeY >= 3) {
            cv::GaussianBlur(matEvent.mat(), output, cv::Size(_kernelSizeX,_kernelSizeY), _sigmaX, _sigmaY, (cv::BorderTypes)_borderType);
        }
        else {
            matEvent.mat().copyTo(output);
        }
        emit out(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported input type");
    }
}
