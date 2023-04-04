#ifndef BilateralFilter_H
#define BilateralFilter_H

#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <QObject>
#include <opencv2/core.hpp>

class BilateralFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Bilateral Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

private:

  Q_PROPERTY(FilterSize FilterSize READ filterSize WRITE filterSize NOTIFY filterSizeChanged)
  Q_PROPERTY(qreal SigmaColor READ sigmaColor WRITE sigmaColor NOTIFY sigmaColorChanged)
  Q_PROPERTY(qreal SigmaSpace READ sigmaSpace WRITE sigmaSpace NOTIFY sigmaSpaceChanged)
  Q_PROPERTY(BorderType BorderType READ borderType WRITE borderType NOTIFY borderTypeChanged)

public:

  enum FilterSize {
    SigmaSpace = 0,
    FilterSize_1 = 1,
    FilterSize_2 = 2,
    FilterSize_3 = 3,
    FilterSize_4 = 4,
    FilterSize_5 = 5,
  };

  Q_ENUM(FilterSize)

  enum BorderType {
    DEFAULT = cv::BORDER_DEFAULT,
    CONSTANT = cv::BORDER_CONSTANT,
    ISOLATED = cv::BORDER_ISOLATED,
    REPLICATE = cv::BORDER_REPLICATE,
    REFLECT = cv::BORDER_REFLECT,
    REFLECT_101 = cv::BORDER_REFLECT_101,
    TRANSPARENT = cv::BORDER_TRANSPARENT,
    WRAP = cv::BORDER_WRAP
  };

  Q_ENUM(BorderType)

  Q_INVOKABLE explicit BilateralFilter(QObject* parent = nullptr);

  virtual ~BilateralFilter() {}

  FilterSize filterSize() const { return _filterSize; }
  void filterSize(FilterSize filterSize) { _filterSize = filterSize; }

  qreal sigmaColor() const { return _sigmaColor; }
  void sigmaColor(qreal sigmaColor) { _sigmaColor = sigmaColor; };

  qreal sigmaSpace() const { return _sigmaSpace; }
  void sigmaSpace(qreal sigmaSpace) { _sigmaSpace = sigmaSpace; };

  BorderType borderType() const { return _borderType; }
  void borderType(BorderType borderType) { _borderType = borderType; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& dstEvent);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

    void filterSizeChanged(FilterSize);
    void sigmaColorChanged(qreal);
    void sigmaSpaceChanged(qreal);
    void borderTypeChanged(BorderType);

protected:

private:

  MatEvent _srcEvent;
  FilterSize _filterSize;
  qreal _sigmaColor;
  qreal _sigmaSpace;
  BorderType _borderType;

};

#endif // BilateralFilter_H
