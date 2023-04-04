#ifndef BitwiseOrOperation_H
#define BitwiseOrOperation_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class BitwiseOr : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Bitwise OR")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

public:

  Q_INVOKABLE explicit BitwiseOr(QObject* parent = nullptr);

  virtual ~BitwiseOr() {}

public slots:

  void src1(const MatEvent& src1Event);
  void src2(const MatEvent& src2Event);
  void mask(const MatEvent& maskEvent);

signals:

  void dst(const MatEvent& dstEvent);

protected:

  void doUpdate() override;

private:

  MatEvent _src1Event;
  MatEvent _src2Event;
  MatEvent _maskEvent;

};

#endif // BitwiseOrOperation_H
