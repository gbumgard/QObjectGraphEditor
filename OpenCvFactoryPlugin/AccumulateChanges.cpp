#include "AccumulateChanges.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AccumulateChanges)

AccumulateChanges::AccumulateChanges(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _noiseThreshold(0)
    , _areaThreshold(0)
    , _filterWeight(1.0)
    , _accumulator()
{
}

void AccumulateChanges::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat sample;
        switch (matEvent.mat().depth()) {
        case CV_8U:
        case CV_8S:
        case CV_32F:
            //sample = matEvent.mat();
            break;
        default:
            //matEvent.mat().convertTo(sample,CV_32F);
            break;
        }
        matEvent.mat().convertTo(sample,CV_32F);

        if (_accumulator.empty() ||
          matEvent.payload().channels() != _accumulator.channels() ||
          matEvent.payload().size() != _accumulator.size()) {
            int alpha = 0;

            switch (matEvent.mat().depth()) {
                case CV_8U:
                case CV_8S:
                case CV_16U:
                case CV_16S:
                    alpha = CV_32F;
                    break;
                case CV_32S:
                case CV_32F:
                default:
                    alpha = CV_64F;
                    break;
            }
            int type = CV_MAKETYPE(alpha,matEvent.mat().channels());
            _accumulator = cv::Mat(matEvent.mat().size(), type, cv::Scalar(0.0));
        }
        cv::Mat absDiff;
        cv::absdiff(sample,_accumulator, absDiff);
        cv::pow(absDiff,2,absDiff);
        cv::Mat updateMask = absDiff > _noiseThreshold;
        int count = cv::countNonZero(updateMask);
        if (count > _areaThreshold) {
            cv::accumulateWeighted(sample, _accumulator, _filterWeight, updateMask);
            cv::Mat accumulatorOutput;
            _accumulator.convertTo(accumulatorOutput,matEvent.mat().type());
            emit out(QVariant::fromValue(MatEvent(accumulatorOutput,matEvent.timestamp())));
        }
        cv::Mat diffOutput;
        absDiff.convertTo(diffOutput,matEvent.mat().type());
        emit diff(QVariant::fromValue(MatEvent(diffOutput,matEvent.timestamp())));
        emit mask(QVariant::fromValue(MatEvent(updateMask,matEvent.timestamp())));
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}
