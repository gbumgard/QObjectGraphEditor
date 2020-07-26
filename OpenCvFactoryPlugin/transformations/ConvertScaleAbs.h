#ifndef CONVERTSCALEABSOPERATION_H
#define CONVERTSCALEABSOPERATION_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class ConvertScaleAbs : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Convert/Scale Absolute")
  Q_CLASSINFO("directory","OpenCV/Transformations")

  Q_PROPERTY(double scale READ scale WRITE scale)
  Q_PROPERTY(double offset READ offset WRITE offset)

public:

  Q_INVOKABLE explicit ConvertScaleAbs(QObject* parent = nullptr);

  virtual ~ConvertScaleAbs() {}

  double scale() const { return _scale; }
  double offset() const { return _offset; }

public slots:

  void in(const MatEvent& alpha);

  void scale(double scale);
  void offset(double offset);

signals:

  void out(const MatEvent& event);

protected:

private:

  double _scale;
  double _offset;
};

#endif // CONVERTSCALEABSOPERATION_H
