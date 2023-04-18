#ifndef BACKGROUNDSUBTRACTORKNN_H
#define BACKGROUNDSUBTRACTORKNN_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <opencv2/video/background_segm.hpp>

class BackgroundSubtractorKNN : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","Background Subtractor KNN")
    Q_CLASSINFO("directory","OpenCV/Image Processing")
    Q_CLASSINFO("slots","in(QVariant)")
    Q_CLASSINFO("signals","out(QVariant),mask(QVariant)")


    Q_PROPERTY(double filterWeight READ filterWeight WRITE setFilterWeight NOTIFY filterWeightChanged)

public:

    Q_INVOKABLE explicit BackgroundSubtractorKNN(QObject* parent = nullptr);

    virtual ~BackgroundSubtractorKNN() {}

    double filterWeight() const { return _filterWeight; }

    void setFilterWeight(double weight) { _filterWeight = weight; }

public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

signals:

    QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

    QVARIANT_PAYLOAD(MatEvent) void mask(const QVariant& dstEvent);

    void filterWeightChanged(double);

private:

    double _filterWeight;
    cv::Ptr<cv::BackgroundSubtractorKNN> _bkgndSubtractor;
};

#endif // BACKGROUNDSUBTRACTORKNN_H
