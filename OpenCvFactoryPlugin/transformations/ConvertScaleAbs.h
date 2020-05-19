#ifndef CONVERTSCALEABSOPERATION_H
#define CONVERTSCALEABSOPERATION_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class ConvertScaleAbs : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Convert/Scale Absolute")
  Q_CLASSINFO("directory","OpenCV/Operations")

  Q_PROPERTY(double scale READ scale WRITE scale)
  Q_PROPERTY(double offset READ offset WRITE offset)

public:

  Q_INVOKABLE explicit ConvertScaleAbs(QObject* parent = nullptr);

  virtual ~ConvertScaleAbs() {}

  double scale() const { return _scale; }
  double offset() const { return _offset; }

public slots:

  void in(const cv::Mat& alpha);

  void scale(double scale);
  void offset(double offset);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  double _scale;
  double _offset;

  cv::Mat _input;

};

#endif // CONVERTSCALEABSOPERATION_H
