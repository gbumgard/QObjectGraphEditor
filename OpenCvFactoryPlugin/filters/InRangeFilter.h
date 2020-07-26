#ifndef InRangeFilter_H
#define InRangeFilter_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class InRangeFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","In Range Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(double minimum READ minimum WRITE minimum)
  Q_PROPERTY(double maximum READ maximum WRITE maximum)
  Q_PROPERTY(double low READ low WRITE low)
  Q_PROPERTY(double high READ high WRITE high)

public:

  Q_INVOKABLE explicit InRangeFilter(QObject* parent = nullptr);

  virtual ~InRangeFilter() {}

  double minimum() const { return _minimum; }
  double maximum() const { return _maximum; }

  double low() const { return _low; }
  double high() const { return _high; }

public slots:

  void in(const MatEvent& event);

  void minimum(double minimum) {
    _minimum = minimum;
  }

  void maximum(double maximum) {
    _maximum = maximum;
  }

  void low(double low) {
    _low = low;
  }

  void high(double high) {
    _high = high;
  }

signals:

  void out(const MatEvent& event);

private:

  double _minimum;
  double _maximum;
  double _low;
  double _high;

};

#endif // InRangeFilter_H
