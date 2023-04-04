#ifndef ACCUMULATECHANGES_H
#define ACCUMULATECHANGES_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class AccumulateChanges : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","AccumulateChanges")
    Q_CLASSINFO("directory","OpenCV/Image Processing")

    Q_PROPERTY(int noiseThreshold READ noiseThreshold WRITE setNoiseThreshold NOTIFY noiseThresholdChanged)
    Q_PROPERTY(int areaThreshold READ areaThreshold WRITE setAreaThreshold NOTIFY areaThresholdChanged)
    Q_PROPERTY(double filterWeight READ filterWeight WRITE setFilterWeight NOTIFY filterWeightChanged)

public:

    Q_INVOKABLE explicit AccumulateChanges(QObject* parent = nullptr);

    virtual ~AccumulateChanges() {}

    int noiseThreshold() const { return _noiseThreshold; }

    int areaThreshold() const { return _areaThreshold; }

    double filterWeight() const { return _filterWeight; }


public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

    void setNoiseThreshold(int threshold) { _noiseThreshold = threshold; }
    void setAreaThreshold(int threshold) { _areaThreshold = threshold; }
    void setFilterWeight(double weight) { _filterWeight = weight; }

signals:

    QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);
    QVARIANT_PAYLOAD(MatEvent) void diff(const QVariant& dstEvent);
    QVARIANT_PAYLOAD(MatEvent) void mask(const QVariant& dstEvent);

    void noiseThresholdChanged(int);
    void areaThresholdChanged(int);
    void filterWeightChanged(double);

private:

    int _noiseThreshold;
    int _areaThreshold;
    double _filterWeight;
    cv::Mat _accumulator;
};

#endif // ACCUMULATECHANGES_H
