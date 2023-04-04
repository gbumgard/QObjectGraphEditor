#ifndef INRANGEOPERATION_H
#define INRANGEOPERATION_H

#include "AbstractOpenCvObject.h"

#include <QObject>
#include <opencv2/core.hpp>

class InRange : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","In Range")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

public:

  Q_INVOKABLE explicit InRange(QObject* parent = nullptr);

  virtual ~InRange() {}

public slots:

  void src(const MatEvent& srcEvent);
  void lowerb(const MatEvent& lowerbEvent);
  void upperb(const MatEvent& upperbEvent);

signals:

  void dst(const MatEvent& mat);

protected:

  void doUpdate();

private:

  MatEvent _srcEvent;
  MatEvent _lowerbEvent;
  MatEvent _upperbEvent;

};

#endif // INRANGEOPERATION_H
