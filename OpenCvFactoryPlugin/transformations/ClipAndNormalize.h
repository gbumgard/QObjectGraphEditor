#ifndef CLIPANDNORMALIZE_H
#define CLIPANDNORMALIZE_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class ClipAndNormalize : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Clip and Normalize")
  Q_CLASSINFO("directory","OpenCV/Transformations")

  Q_PROPERTY(double minimum READ minimum WRITE minimum)
  Q_PROPERTY(double maximum READ maximum WRITE maximum)
  Q_PROPERTY(double range READ range WRITE range)
  Q_PROPERTY(double offset READ offset WRITE offset)

public:

  Q_INVOKABLE explicit ClipAndNormalize(QObject* parent = nullptr);

  virtual ~ClipAndNormalize() {}

  double minimum() const { return _minimum; }
  double maximum() const { return _maximum; }
  double range() const { return _range; }
  double offset() const { return _offset; }

public slots:

  void in(const TaggedMat& mat);

  void minimum(double value);
  void maximum(double value);
  void range(double value);
  void offset(double value);

signals:

  void out(const TaggedMat& mat);

protected:

  double _minimum;
  double _maximum;
  double _range;
  double _offset;

};

#endif // CLIPANDNORMALIZE_H
