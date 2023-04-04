#ifndef InRangeFilter_H
#define InRangeFilter_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class InRangeFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","In Range Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(double minimum READ minimum WRITE minimum NOTIFY minimumChanged)
  Q_PROPERTY(double maximum READ maximum WRITE maximum NOTIFY maximumChanged)
  Q_PROPERTY(double low READ low WRITE low NOTIFY lowChanged)
  Q_PROPERTY(double high READ high WRITE high NOTIFY highChanged)

public:

  Q_INVOKABLE explicit InRangeFilter(QObject* parent = nullptr);

  virtual ~InRangeFilter() {}

  double minimum() const { return _minimum; }
  double maximum() const { return _maximum; }

  double low() const { return _low; }
  double high() const { return _high; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

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

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void minimumChanged(double);
  void maximumChanged(double);
  void lowChanged(double);
  void highChanged(double);

private:

  double _minimum;
  double _maximum;
  double _low;
  double _high;

};

#endif // InRangeFilter_H
