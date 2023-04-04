#ifndef AccumulateWeighted_H
#define AccumulateWeighted_H

#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <opencv2/core.hpp>

class AccumulateWeighted : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Accumulate Weighted")
  Q_CLASSINFO("directory","OpenCV/Image Processing/Motion Analysis")

  Q_PROPERTY(double alpha READ alpha WRITE alpha NOTIFY alphaChanged)

public:

  Q_INVOKABLE explicit AccumulateWeighted(QObject* parent = nullptr);

  virtual ~AccumulateWeighted() {}

  double alpha() const { return _alpha; }

  void alpha(double alpha);

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

  QVARIANT_PAYLOAD(MatEvent) void mask(const QVariant& srcEvent);

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void alphaChanged(double);

protected:

  virtual void doUpdate() override;

private:

  double _alpha;

  cv::Mat _accumulator;
  MatEvent _srcEvent;
  MatEvent _maskEvent;

};

#endif // AccumulateWeighted_H
