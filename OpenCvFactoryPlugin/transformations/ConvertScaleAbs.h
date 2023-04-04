#ifndef CONVERTSCALEABSOPERATION_H
#define CONVERTSCALEABSOPERATION_H

#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <opencv2/core.hpp>

class ConvertScaleAbs : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Convert/Scale Absolute")
  Q_CLASSINFO("directory","OpenCV/Transformations")

  Q_PROPERTY(double scale READ scale WRITE scale NOTIFY scaleChanged)
  Q_PROPERTY(double offset READ offset WRITE offset NOTIFY offsetChanged)

public:

  Q_INVOKABLE explicit ConvertScaleAbs(QObject* parent = nullptr);

  virtual ~ConvertScaleAbs() {}

  double scale() const { return _scale; }
  double offset() const { return _offset; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

  void scale(double scale);
  void offset(double offset);

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void scaleChanged(double);
  void offsetChanged(double);

protected:

private:

  double _scale;
  double _offset;
};

#endif // CONVERTSCALEABSOPERATION_H
