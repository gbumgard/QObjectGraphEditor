#ifndef ABSDIFFERENCE_H
#define ABSDIFFERENCE_H

#include "AbstractOpenCvObject.h"

#include <QObject>

#include <opencv2/core.hpp>

class AbsDifference : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Absolute Difference")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

public:

  Q_INVOKABLE explicit AbsDifference(QObject* parent = nullptr);

  virtual ~AbsDifference() {}

public slots:

  void src1(const MatEvent& src1Event);
  void src2(const MatEvent& src2Event);

signals:

  void dst(const MatEvent& dstEvent);

protected:

  void doUpdate() override;

private:

  MatEvent _src1Event;
  MatEvent _src2Event;

};

#endif // ABSDIFFERENCE_H
