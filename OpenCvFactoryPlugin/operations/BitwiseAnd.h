#ifndef BitwiseAnd_H
#define BitwiseAnd_H

#include "AbstractOpenCvObject.h"
#include <QObject>
#include <opencv2/core.hpp>

class BitwiseAnd : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Bitwise AND")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

public:

  Q_INVOKABLE explicit BitwiseAnd(QObject* parent = nullptr);

  virtual ~BitwiseAnd() {}

public slots:

  void src1(const MatEvent& src1Event);
  void src2(const MatEvent& src2Event);
  void mask(const MatEvent& maskEvent);

signals:

  void dst(const MatEvent& dstEvent);

protected:

  virtual void doUpdate() override;

private:

  MatEvent _src1Event;
  MatEvent _src2Event;
  MatEvent _maskEvent;

};

#endif // BitwiseAnd_H
