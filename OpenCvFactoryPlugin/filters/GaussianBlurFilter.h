#ifndef GAUSSIANBLURFILTER_H
#define GAUSSIANBLURFILTER_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <QSize>

class GaussianBlurFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Gaussian Blur")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

  enum BorderType {
    BORDER_CONSTANT = cv::BORDER_CONSTANT,
    BORDER_REPLICATE = cv::BORDER_REPLICATE,
    BORDER_REFLECT = cv::BORDER_REFLECT,
    BORDER_WRAP = cv::BORDER_WRAP,
    BORDER_REFLECT_101 = cv::BORDER_REFLECT_101,
    BORDER_REFLECT101 = cv::BORDER_REFLECT101,
    BORDER_DEFAULT = cv::BORDER_DEFAULT,
  };

  Q_ENUM(BorderType)

private:

  Q_PROPERTY(int kernelSizeX READ kernelSizeX WRITE setKernelSizeX)
  Q_PROPERTY(int kernelSizeY READ kernelSizeY WRITE setKernelSizeY)
  Q_PROPERTY(double sigmaX READ sigmaX WRITE setSigmaX)
  Q_PROPERTY(double sigmaY READ sigmaY WRITE setSigmaY)
  Q_PROPERTY(BorderType borderType READ borderType WRITE setBorderType)

public:

  Q_INVOKABLE explicit GaussianBlurFilter(QObject* parent = nullptr);

  virtual ~GaussianBlurFilter() {}

  int kernelSizeX() const { return _kernelSizeX; }
  int kernelSizeY() const { return _kernelSizeY; }

  double sigmaX() const { return _sigmaX; }
  double sigmaY() const { return _sigmaY; }

  BorderType borderType() const { return _borderType; }
  void setBorderType(BorderType borderType);

public slots:

  void in(const cv::Mat& mat);

  void setKernelSizeX(int kernelSizeX);
  void setKernelSizeY(int kernelSizeY);

  void setSigmaX(double sigmaX);
  void setSigmaY(double sigmaY);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  int _kernelSizeX;
  int _kernelSizeY;

  double _sigmaX;
  double _sigmaY;

  BorderType _borderType;

  cv::Mat _input;

};

#endif // GAUSSIANBLURFILTER_H
