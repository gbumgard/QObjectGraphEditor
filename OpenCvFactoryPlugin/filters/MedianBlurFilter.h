#ifndef MEDIANFILTER_H
#define MEDIANFILTER_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include <opencv2/core.hpp>

class MedianBlurFilter : public QObject
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

  void in(const MatEvent& mat);

  void setAperatureSize(AperatureSize aperatureSize);

signals:

  void out(const MatEvent& output);

protected:


private:

  AperatureSize _aperatureSize;

};

#endif // MEDIANFILTER_H
