#ifndef ADDOPERATION_H
#define ADDOPERATION_H

#include "AbstractOpenCvObject.h"
#include <QObject>
#include <opencv2/core.hpp>

class AddArray : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Add Array")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

  Q_PROPERTY(MatDepth dtype READ dtype WRITE dtype)

public:

  enum MatDepth {
    Depth_SRC = -1,
    Depth_8U = CV_8U,
    Depth_8S = CV_8S,
    Depth_16U = CV_16U,
    Depth_16S = CV_16S,
    Depth_32S = CV_32S,
    Depth_32F = CV_32F,
    Depth_64F = CV_64F,
    Depth_16F = CV_16F
  };

  Q_ENUM(MatDepth)

  Q_INVOKABLE explicit AddArray(QObject* parent = nullptr);

  virtual ~AddArray() {}

  MatDepth dtype() const { return _dtype; }
  void dtype(MatDepth dtype) { _dtype = dtype; }

public slots:

  void src1(const MatEvent& src1Event);
  void src2(const MatEvent& src2Event);
  void mask(const MatEvent& maskEvent);

signals:

  void dst(const MatEvent& mat);

protected:

  virtual void doUpdate() override;

private:

  MatDepth _dtype;

  MatEvent _src1Event;
  MatEvent _src2Event;
  MatEvent _maskEvent;

};

#endif // ADDOPERATION_H
