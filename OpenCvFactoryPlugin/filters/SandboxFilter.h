#ifndef SandboxFilter_H
#define SandboxFilter_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/bgsegm.hpp>

class SandboxFilter : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","Sandbox Filter")
    Q_CLASSINFO("directory","OpenCV/Filters")
    Q_CLASSINFO("slots","in(QVariant)")
    Q_CLASSINFO("signals","out(QVariant),norm(QVariant)")

    Q_PROPERTY(int rangeMin READ rangeMin WRITE rangeMin NOTIFY rangeMinChanged)
    Q_PROPERTY(int rangeMax READ rangeMax WRITE rangeMax NOTIFY rangeMaxChanged)
    Q_PROPERTY(int filterThreshold READ filterThreshold WRITE filterThreshold NOTIFY filterThresholdChanged)
    Q_PROPERTY(double slowFilterWeight READ slowFilterWeight WRITE slowFilterWeight NOTIFY slowFilterWeightChanged)
    Q_PROPERTY(double fastFilterWeight READ fastFilterWeight WRITE fastFilterWeight NOTIFY fastFilterWeightChanged)
    Q_PROPERTY(int minPixelStability READ minPixelStability WRITE minPixelStability NOTIFY minPixelStabilityChanged)

public:

    Q_INVOKABLE explicit SandboxFilter(QObject* parent = nullptr);

    virtual ~SandboxFilter() {}

    int rangeMin() const { return _rangeMin; }
    int rangeMax() const { return _rangeMax; }
    double filterThreshold() const { return  _filterThreshold; }
    double slowFilterWeight() const { return  _slowFilterWeight; }
    double fastFilterWeight() const { return  _fastFilterWeight; }
    int minPixelStability() const { return _minPixelStability; }

    void rangeMin(int value) { _rangeMin = value; }
    void rangeMax(int value) { _rangeMax = value; }
    void filterThreshold(int value) { _filterThreshold = value; }
    void slowFilterWeight(double value) { _slowFilterWeight = value; }
    void fastFilterWeight(double value) { _fastFilterWeight = value; }
    void minPixelStability(int value) { _minPixelStability = value; }

public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

signals:

    QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void norm(const QVariant& event);

    void rangeMinChanged(double);
    void rangeMaxChanged(double);
    void filterThresholdChanged(int);
    void slowFilterWeightChanged(double);
    void fastFilterWeightChanged(double);
    void minPixelStabilityChanged(int);

private:

    int _rangeMin;
    int _rangeMax;
    int _filterThreshold;
    double _slowFilterWeight;
    double _fastFilterWeight;
    int _minPixelStability;

    cv::Mat _slowAccumulator;
    cv::Mat _fastAccumulator;
    cv::Mat _stabilityCount;
    cv::Mat _output;
};

#endif // SandboxFilter_H
