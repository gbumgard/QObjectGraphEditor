#ifndef BACKGROUNDSUBTRACTORCNT_H
#define BACKGROUNDSUBTRACTORCNT_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <opencv2/bgsegm.hpp>

class BackgroundSubtractorCNT : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","Background Subtractor CNT")
    Q_CLASSINFO("directory","OpenCV/Image Processing")
    Q_CLASSINFO("slots","in(QVariant)")
    Q_CLASSINFO("signals","inFore(QVariant),inBack(QVariant),mask(QVariant),bkgnd(QVariant)")


    Q_PROPERTY(double learningRate READ learningRate WRITE learningRate NOTIFY learningRateChanged)
    Q_PROPERTY(bool useHistory READ useHistory WRITE useHistory NOTIFY useHistoryChanged)
    Q_PROPERTY(int minPixelStability READ minPixelStability WRITE minPixelStability NOTIFY minPixelStabilityChanged)
    Q_PROPERTY(int maxPixelStability READ maxPixelStability WRITE maxPixelStability NOTIFY maxPixelStabilityChanged)

public:

    Q_INVOKABLE explicit BackgroundSubtractorCNT(QObject* parent = nullptr);

    virtual ~BackgroundSubtractorCNT() {}

    double learningRate() const { return _learningRate; }
    void learningRate(double rate) { _learningRate = rate; }

    bool useHistory() { return _useHistory; }
    void useHistory(bool value) { _useHistory = value; }

    int minPixelStability() { return _minPixelStability; }
    void minPixelStability(int value) { _minPixelStability = value; }

    int maxPixelStability() { return _maxPixelStability; }
    void maxPixelStability(int value) { _maxPixelStability = value; }

public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

signals:

    QVARIANT_PAYLOAD(MatEvent) void inFore(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void inBack(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void mask(const QVariant& event);
    QVARIANT_PAYLOAD(MatEvent) void bkgnd(const QVariant& event);

    void learningRateChanged(double);
    void useHistoryChanged(bool);
    void minPixelStabilityChanged(int);
    void maxPixelStabilityChanged(int);

private:

    double _learningRate;
    bool _useHistory;
    int _minPixelStability;
    int _maxPixelStability;

    cv::Ptr<cv::bgsegm::BackgroundSubtractorCNT> _bkgndSubtractor;
};

#endif // BACKGROUNDSUBTRACTORCNT_H
