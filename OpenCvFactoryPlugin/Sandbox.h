#ifndef Sandbox_H
#define Sandbox_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/bgsegm.hpp>

class Sandbox : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","Old Sandbox Filter")
    Q_CLASSINFO("directory","OpenCV/Filters")
    Q_CLASSINFO("slots","in(QVariant)")
    //Q_CLASSINFO("signals","out(QVariant),norm(QVariant),mask(QVariant),fgnd(QVariant),bgnd(QVariant),mean(QVariant),var(QVariant),diff(QVariant)")

    Q_PROPERTY(int min READ min WRITE min NOTIFY minChanged)
    Q_PROPERTY(int max READ max WRITE max NOTIFY maxChanged)
    Q_PROPERTY(double filterWeight READ filterWeight WRITE filterWeight NOTIFY filterWeightChanged)
    Q_PROPERTY(int maxPixelStability READ maxPixelStability WRITE maxPixelStability NOTIFY maxPixelStabilityChanged)
    Q_PROPERTY(int minPixelStability READ minPixelStability WRITE minPixelStability NOTIFY minPixelStabilityChanged)
    Q_PROPERTY(bool useHistory READ useHistory WRITE useHistory NOTIFY useHistoryChanged)
    Q_PROPERTY(double slowFilterWeight READ slowFilterWeight WRITE slowFilterWeight NOTIFY slowFilterWeightChanged)
    Q_PROPERTY(double fastFilterWeight READ fastFilterWeight WRITE fastFilterWeight NOTIFY fastFilterWeightChanged)
    Q_PROPERTY(int filterThreshold READ filterThreshold WRITE filterThreshold NOTIFY filterThresholdChanged)

public:

    Q_INVOKABLE explicit Sandbox(QObject* parent = nullptr);

    virtual ~Sandbox() {}

    int min() const { return _min; }
    int max() const { return _max; }
    double filterWeight() const { return  _filterWeight; }
    double slowFilterWeight() const { return  _slowFilterWeight; }
    double fastFilterWeight() const { return  _fastFilterWeight; }
    double filterThreshold() const { return  _filterThreshold; }
    int minPixelStability() const { return _minPixelStability; }
    int maxPixelStability() const { return _maxPixelStability; }
    bool useHistory() const { return _useHistory; }

    void min(int value) {
        _min = value;
    }

    void max(int value) {
        _max = value;
    }

    void filterWeight(double weight)
    {
        _filterWeight = weight;
    }

    void minPixelStability(int value) {
        _minPixelStability = value;
    }

    void maxPixelStability(int value) {
        _maxPixelStability = value;
    }

    void useHistory(bool value) {
        _useHistory = value;
    }

    void slowFilterWeight(double weight)
    {
        _slowFilterWeight = weight;
    }

    void fastFilterWeight(double weight)
    {
        _fastFilterWeight = weight;
    }


    void filterThreshold(int value) {
        _filterThreshold = value;
    }

public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

signals:

    QVARIANT_PAYLOAD(MatEvent) void fgnd(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void bgnd(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void norm(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void mask(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void mean(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void var(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void diff(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void fmask(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void umask(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void dmask(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void stability(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void smooth(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void final(const QVariant& event);

    void minChanged(double);
    void maxChanged(double);
    void filterWeightChanged(double);
    void minPixelStabilityChanged(int);
    void maxPixelStabilityChanged(int);
    void useHistoryChanged(bool);
    void slowFilterWeightChanged(double);
    void fastFilterWeightChanged(double);
    void filterThresholdChanged(int);

private:

    int _min;
    int _max;
    double _filterWeight;
    int _minPixelStability;
    int _maxPixelStability;
    bool _useHistory;
    cv::Ptr<cv::bgsegm::BackgroundSubtractorCNT> _bkgndSubtractor;
    std::vector<cv::Mat> _frameBuffer;

    double _slowFilterWeight;
    double _fastFilterWeight;

    cv::Mat _slowAccumulator;
    cv::Mat _fastAccumulator;
    int _filterThreshold;
    cv::Mat _stabilityCount;
    cv::Mat _output;
};

#endif // Sandbox_H
