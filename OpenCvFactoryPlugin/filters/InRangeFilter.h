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
  Q_CLASSINFO("slots","in(QVariant)")
  Q_CLASSINFO("signals","out(QVariant)")

  Q_PROPERTY(double minimum READ minimum WRITE minimum NOTIFY minimumChanged)
  Q_PROPERTY(double maximum READ maximum WRITE maximum NOTIFY maximumChanged)

public:

  Q_INVOKABLE explicit InRangeFilter(QObject* parent = nullptr);

  virtual ~InRangeFilter() {}

  double minimum() const { return _minimum; }
  double maximum() const { return _maximum; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

  void minimum(double minimum) {
    _minimum = minimum;
  }

  void maximum(double maximum) {
    _maximum = maximum;
  }
signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

  void minimumChanged(double);
  void maximumChanged(double);

private:

  double _minimum;
  double _maximum;

};

#endif // InRangeFilter_H
