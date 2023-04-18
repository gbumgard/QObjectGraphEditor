#include "SandboxFilter.h"
#include "ObjectModel.h"

#include <opencv2/imgproc.hpp>

static cv::Mat getStructuringElement(cv::MorphShapes shape, int kernelSize, cv::Point anchor)
{
    int i, j;
    int r = 0, c = 0;
    double inv_r2 = 0;

    cv::Size ksize(kernelSize,kernelSize);

    CV_Assert( shape == cv::MORPH_RECT || shape == cv::MORPH_CROSS || shape == cv::MORPH_ELLIPSE );

    if (anchor.x < 0) anchor.x = kernelSize >> 1;
    if (anchor.y < 0) anchor.y = kernelSize >> 1;

    if( ksize == cv::Size(1,1) )
        shape = cv::MORPH_RECT;

    if( shape == cv::MORPH_ELLIPSE )
    {
        r = ksize.height/2;
        c = ksize.width/2;
        inv_r2 = r ? 1./((double)r*r) : 0;
    }

    cv::Mat elem(ksize, CV_8U);

    for( i = 0; i < ksize.height; i++ )
    {
        uchar* ptr = elem.ptr(i);
        int j1 = 0, j2 = 0;

        if( shape == cv::MORPH_RECT || (shape == cv::MORPH_CROSS && i == anchor.y) )
            j2 = ksize.width;
        else if( shape == cv::MORPH_CROSS )
            j1 = anchor.x, j2 = j1 + 1;
        else
        {
            int dy = i - r;
            if( std::abs(dy) <= r )
            {
                int dx = cv::saturate_cast<int>(c*std::sqrt((r*r - dy*dy)*inv_r2));
                j1 = std::max( c - dx, 0 );
                j2 = std::min( c + dx + 1, ksize.width );
            }
        }

        for( j = 0; j < j1; j++ )
            ptr[j] = 0;
        for( ; j < j2; j++ )
            ptr[j] = 1;
        for( ; j < ksize.width; j++ )
            ptr[j] = 0;
    }

    return elem;
}

REGISTER_CLASS(SandboxFilter)

SandboxFilter::SandboxFilter(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _rangeMin(500) // millimeters
    , _rangeMax(1000) // millimeters
    , _filterThreshold(1)
    , _slowFilterWeight(.05)
    , _fastFilterWeight(.80)
    , _minPixelStability(15)
{
}

void SandboxFilter::in(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        cv::Mat input(matEvent.mat());

#if 1
        // Scale the depth image to map the pixel values
        // between _rangeMin and _rangeMax to the 8-bit range [0,255].
        // The range for a sandbox might be 500 (mm) to 1000 (mm).
        //double inputRange = _rangeMax - _rangeMin;
        //double scale = 255.0 / inputRange;
        cv::Mat aboveLowerMask = input >= _rangeMin;
        cv::Mat belowUpperMask = input <= _rangeMax;
        cv::Mat inRangeMask = aboveLowerMask & belowUpperMask;
        //cv::Mat output = input.clone();
        //output -= _rangeMin;
        //output *= scale;
        //output.copyTo(output, inRangeMask);
        //output.convertTo(output,CV_8U);
        cv::Mat zeroStructuringElement = getStructuringElement(cv::MORPH_ELLIPSE, 3, cv::Point(-1,-1));
        cv::morphologyEx(input,input,cv::MORPH_CLOSE,zeroStructuringElement,cv::Point(-1,-1),1);

        emit norm(QVariant::fromValue(MatEvent(inRangeMask,matEvent.timestamp())));
        //emit norm(QVariant::fromValue(MatEvent(output & output > 0,matEvent.timestamp())));

#endif

        if (_slowAccumulator.empty() ||
            input.channels() != _slowAccumulator.channels() ||
            input.size() != _slowAccumulator.size()) {

            _slowAccumulator = cv::Mat(input.size(), CV_64FC1, cv::Scalar(0.0));
            _stabilityCount= cv::Mat(input.size(), CV_16UC1, cv::Scalar(0));
        }

        if (_fastAccumulator.empty() ||
            input.channels() != _fastAccumulator.channels() ||
            input.size() != _fastAccumulator.size()) {

            _fastAccumulator = cv::Mat(input.size(), CV_64FC1, cv::Scalar(0.0));
        }

        cv::accumulateWeighted(input, _slowAccumulator, _slowFilterWeight, inRangeMask);
        cv::accumulateWeighted(input, _fastAccumulator, _fastFilterWeight, inRangeMask);
        //cv::accumulateWeighted(_slowAccumulator, _fastAccumulator, _fastFilterWeight, ~inRangeMask);

        cv::Mat absDiff;
        cv::absdiff(_slowAccumulator,_fastAccumulator,absDiff);

        cv::Mat filterMask = absDiff > _filterThreshold;
        cv::Mat zeros = cv::Mat::zeros(input.size(),CV_16U);
        zeros.copyTo(_stabilityCount,filterMask);
        cv::Mat ones = cv::Mat::ones(input.size(),CV_16U);
        cv::Mat updateMask = (_stabilityCount < _minPixelStability);
        cv::copyTo(updateMask,updateMask,~filterMask);
        cv::add(_stabilityCount,ones,_stabilityCount,updateMask);
        cv::Mat dispMask = _stabilityCount >= (_minPixelStability - 0);

        cv::accumulateWeighted(input, _slowAccumulator, 1.0, filterMask);
        //cv::Mat smoothed;
        //_slowAccumulator.convertTo(smoothed,CV_8U);

        if (_output.empty() ||
            input.channels() != _output.channels() ||
            input.size() != _output.size()) {

            _output= cv::Mat(input.size(), input.type(), cv::Scalar(0));
        }
        //cv::Mat dispStructuringElement = getStructuringElement(cv::MORPH_ELLIPSE, 3, cv::Point(-1,-1));
        //cv::morphologyEx(dispMask,dispMask,cv::MORPH_CLOSE,dispStructuringElement,cv::Point(-1,-1),1);
        cv::copyTo(_slowAccumulator,_output,dispMask); // & zeroMask);
        //cv::copyTo(smoothed,_output,dispMask);
        cv::Mat outStructuringElement = getStructuringElement(cv::MORPH_ELLIPSE, 3, cv::Point(-1,-1));
        cv::morphologyEx(_output,_output,cv::MORPH_CLOSE,outStructuringElement,cv::Point(-1,-1),1);
        //cv::Mat final;
        //cv::medianBlur(_output,final,3);
        emit out(QVariant::fromValue(MatEvent(_output,matEvent.timestamp())));

    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported input type");
    }
}
