#ifndef MEDIANFILTER_H
#define MEDIANFILTER_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MedianBlurFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Median Blur Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

private:

  Q_PROPERTY(AperatureSize aperatureSize READ aperatureSize WRITE setAperatureSize)

public:

  enum AperatureSize {
    Aperature3x3 = 3,
    Aperature5x5 = 5,
    Aperature7x7 = 7,
    Aperature9x9 = 9,
    Aperature11x11 = 11
  };

  Q_ENUM(AperatureSize)

  Q_INVOKABLE explicit MedianBlurFilter(QObject* parent = nullptr);

  virtual ~MedianBlurFilter() {}

  AperatureSize aperatureSize() const { return _aperatureSize; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

  void setAperatureSize(AperatureSize aperatureSize);

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

protected:


private:

  AperatureSize _aperatureSize;

};

#endif // MEDIANFILTER_H
