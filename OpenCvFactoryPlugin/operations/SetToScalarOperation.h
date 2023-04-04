#ifndef SETTOSCALAROPERATION_H
#define SETTOSCALAROPERATION_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class SetToScalarOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Set To Scalar Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

  Q_PROPERTY(double value READ value WRITE value)

public:

  Q_INVOKABLE explicit SetToScalarOperation(QObject* parent = nullptr);

  virtual ~SetToScalarOperation() {}

  double value() const { return _value; }

public slots:

  void in(const cv::Mat& mat);
  void value(double value);
  void mask(const cv::Mat& mask);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _input;
  double _value;
  cv::Mat _mask;

};

#endif // SETTOSCALAROPERATION_H
