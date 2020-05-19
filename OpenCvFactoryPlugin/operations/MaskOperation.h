#ifndef MASKOPERATION_H
#define MASKOPERATION_H

#include <QObject>

#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MaskOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mask Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

public:

  Q_INVOKABLE explicit MaskOperation(QObject* parent = nullptr);

  virtual ~MaskOperation() {}

public slots:

  void in(const cv::Mat& mat);
  void mask(const cv::Mat& mat);

signals:

  void out(const cv::Mat& mat);
  void maskUsed(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _input;
  cv::Mat _mask;

};

#endif // MASKOPERATION_H
