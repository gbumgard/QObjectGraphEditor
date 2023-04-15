#ifndef ABSDIFFERENCE_H
#define ABSDIFFERENCE_H

#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"

#include <opencv2/core.hpp>

class AbsDifference : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Absolute Difference")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")
  Q_CLASSINFO("slots","in1(QVariant),in2(QVariant)")
  Q_CLASSINFO("signals","out(QVariant)")

public:

  Q_INVOKABLE explicit AbsDifference(QObject* parent = nullptr);

  virtual ~AbsDifference() {}

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in1(const QVariant& eventIn);
  QVARIANT_PAYLOAD(MatEvent) void in2(const QVariant& eventIn);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& eventOut);

protected:

  void doUpdate() override;

private:

  MatEvent _src1Event;
  cv::Mat _mat1;
  MatEvent _src2Event;
  cv::Mat _mat2;

};

#endif // ABSDIFFERENCE_H
