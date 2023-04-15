#include "AccumulateChanges.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(AccumulateChanges)

AccumulateChanges::AccumulateChanges(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _noiseThreshold(0)
    , _areaThreshold(0)
    , _filterWeight(1.0)
    , _keyFrameInterval(0)
    , _framesUntilNextUpdate(0)
    , _accumulator()
{
}

void AccumulateChanges::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat input;
        if (matEvent.mat().depth() != CV_32F) {
            matEvent.mat().convertTo(input,CV_32F);
        }
        else {
            input = matEvent.mat();
        }

        if (_output.empty()) {
            input.copyTo(_output);
        }

#if 0
        if (_accumulator.empty() ||
          input.channels() != _accumulator.channels() ||
          input.size() != _accumulator.size()) {
            int alpha = 0;

            switch (input.depth()) {
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
        cv::Mat accumulator;
        _accumulator.convertTo(accumulator,input.type());
        cv::absdiff(input,accumulator, absDiff);
        //cv::pow(absDiff,2,absDiff);
#endif

        cv::Mat absDiff;
        cv::Mat output;
        cv::absdiff(input,_output, absDiff);
        absDiff.convertTo(absDiff,input.type());
        cv::Mat thresholdMask = absDiff > _noiseThreshold;
        thresholdMask.convertTo(thresholdMask,input.type());
        int count = cv::countNonZero(thresholdMask);
        if (count > _areaThreshold) {
            //cv::accumulateWeighted(input, _accumulator, _filterWeight, thresholdMask);
            _output = (input & thresholdMask) + (_output & ~thresholdMask);
            //_framesUntilNextUpdate = _keyFrameInterval;
        }
        cv::Mat diffOutput;
        emit out(QVariant::fromValue(MatEvent(_output,matEvent.timestamp())));
        absDiff.convertTo(diffOutput,matEvent.mat().type());
        emit diff(QVariant::fromValue(MatEvent(diffOutput,matEvent.timestamp())));
        emit mask(QVariant::fromValue(MatEvent(thresholdMask,matEvent.timestamp())));
#if 0
        else if (_keyFrameInterval != 0 && _framesUntilNextUpdate == 0) {
            output = input;
            //cv::accumulateWeighted(input, _accumulator, _filterWeight);
            _framesUntilNextUpdate = _keyFrameInterval;
        }
        else if (_keyFrameInterval != 0) {
            _framesUntilNextUpdate--;
        }
        cv::Mat accumulatorOutput;
        cv::Mat diffOutput;
        absDiff.convertTo(diffOutput,matEvent.mat().type());
        emit diff(QVariant::fromValue(MatEvent(diffOutput,matEvent.timestamp())));
        emit mask(QVariant::fromValue(MatEvent(thresholdMask,matEvent.timestamp())));
#endif
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}
