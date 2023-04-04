#ifndef CLIPANDNORMALIZE_H
#define CLIPANDNORMALIZE_H

#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <opencv2/core.hpp>

class ClipAndNormalize : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Clip and Normalize")
  Q_CLASSINFO("directory","OpenCV/Transformations")
  Q_CLASSINFO("slot-order","src(QVariant),min(double),max(double)")
  Q_CLASSINFO("signal-order","dst(QVariant)")

  Q_PROPERTY(double Minimum READ min WRITE min NOTIFY minChanged STORED true)
  Q_PROPERTY(double Maximum READ max WRITE max NOTIFY maxChanged STORED true)
  Q_PROPERTY(double Range READ range WRITE range STORED true)
  Q_PROPERTY(double Offset READ offset WRITE offset STORED true)

public:

  Q_INVOKABLE explicit ClipAndNormalize(QObject* parent = nullptr);

  virtual ~ClipAndNormalize() {}

  double min() const { return _minimum; }
  double max() const { return _maximum; }

  double range() const { return _range; }
  void range(double value);

  double offset() const { return _offset; }
  void offset(double value);

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

  void min(double value);
  void max(double value);

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void minChanged(double min);
  void maxChanged(double max);

protected:

  double _minimum;
  double _maximum;
  double _range;
  double _offset;
};

#endif // CLIPANDNORMALIZE_H
