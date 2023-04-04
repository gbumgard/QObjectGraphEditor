#ifndef BitwiseNot_H
#define BitwiseNot_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>

#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class BitwiseNot : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Bitwise NOT")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

public:

  Q_INVOKABLE explicit BitwiseNot(QObject* parent = nullptr);

  virtual ~BitwiseNot() {}

public slots:

  void src(const MatEvent& srcEvent);
  void mask(const MatEvent& srcEvent);

signals:

  void dst(const MatEvent& dstEvent);

protected:

  void update();

private:

  MatEvent _srcEvent;
  MatEvent _maskEvent;

};

#endif // BitwiseNot_H
