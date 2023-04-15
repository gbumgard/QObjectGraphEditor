#include "TemporalMedianFilterGpu.h"
#include "ObjectModel.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>

REGISTER_CLASS(TemporalMedianFilterGpu)

TemporalMedianFilterGpu::TemporalMedianFilterGpu(QObject* parent)
    : AbstractOpenCvObject(parent)
    , _aperatureSize(Kernel3x3x3)
{
}

void TemporalMedianFilterGpu::setAperatureSize(AperatureSize aperatureSize) {
    _aperatureSize = aperatureSize;
}

void TemporalMedianFilterGpu::in(const QVariant &variant) {

    if (variant.userType() == MatEvent::userType()) {

        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);

        cv::cuda::GpuMat gpuFrame;
        gpuFrame.upload(matEvent.mat());

        while (_frameBuffer.size() > (size_t)_aperatureSize - 1) {
            _frameBuffer.erase(_frameBuffer.begin());
        }

        _frameBuffer.push_back(gpuFrame.clone());

        if (_frameBuffer.size() < _aperatureSize) {

            while (_frameBuffer.size() < _aperatureSize) {
                _frameBuffer.push_back(gpuFrame.clone());
            }
        }

        cv::cuda::GpuMat gpuMedian;
        cv::cuda::reduce(_frameBuffer[0], gpuMedian, 0, cv::REDUCE_AVG);
        cv::Mat median;
        gpuMedian.download(median);

        emit out(QVariant::fromValue(MatEvent(median,matEvent.timestamp())));

    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported input type");
    }
}
