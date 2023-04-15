#ifndef TemporalMedianFilterGps_H
#define TemporalMedianFilterGps_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>

class TemporalMedianFilterGpu : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","Temporal Median Filter GPU")
    Q_CLASSINFO("directory","OpenCV/Filters")

public:

private:

    Q_PROPERTY(AperatureSize aperatureSize READ aperatureSize WRITE setAperatureSize)

public:

    enum AperatureSize {
        Kernel3x3x3 = 3,
        Kernel5x5x5 = 5
    };

    Q_ENUM(AperatureSize)

    Q_INVOKABLE explicit TemporalMedianFilterGpu(QObject* parent = nullptr);

    virtual ~TemporalMedianFilterGpu() {}

    AperatureSize aperatureSize() const { return _aperatureSize; }

    void setAperatureSize(AperatureSize aperatureSize);

public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& dstEvent);


signals:

    QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

protected:

private:

    AperatureSize _aperatureSize;
    std::vector<cv::cuda::GpuMat> _frameBuffer;
};

#endif // TemporalMedianFilterGpu_H
