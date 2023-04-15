#include "operations/AbsDifference.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AbsDifference)

AbsDifference::AbsDifference(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _src1Event(cv::Mat(),MatEvent::UNTIMED)
  , _src2Event(cv::Mat(),MatEvent::UNTIMED)
{
}

void AbsDifference::in1(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        _src1Event = matEvent;
        matEvent.mat().convertTo(_mat1,CV_32F);
#if 0
        if (!_src1Event.isSynchronizedMatch(_src2Event)) {
            // The current _src2 cannot be used with _src1 so we release it.
            _src2Event.release();
            return;
        }
#endif
        updateObject();
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}


void AbsDifference::in2(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        _src2Event = matEvent;
        matEvent.mat().convertTo(_mat2,CV_32F);
#if 0
        if (!_src2Event.isSynchronizedMatch(_src1Event)) {
            // The current _src1 cannot be used with _src2 so we release it.
            _src1Event.release();
            return;
        }
#endif
        updateObject();
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

void AbsDifference::doUpdate() {
  if (!_src1Event.mat().empty() && !_src2Event.mat().empty()) {
    cv::Mat output; //(_mat1.size(),_mat1.depth(),cv::Scalar(0));
    cv::absdiff(_mat1, _mat2, output);
    emit out(QVariant::fromValue(MatEvent(output,MatEvent::maxTimestamp(_src1Event, _src2Event))));
  }
}
