#ifndef GAUSSIANBLURFILTER_H
#define GAUSSIANBLURFILTER_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include <opencv2/core.hpp>
#include <QSize>

class GaussianBlurFilter : public QObject
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

  Q_PROPERTY(int KernelSizeX READ kernelSizeX WRITE kernelSizeX)
  Q_PROPERTY(int KernelSizeY READ kernelSizeY WRITE kernelSizeY)
  Q_PROPERTY(double SigmaX READ sigmaX WRITE sigmaX)
  Q_PROPERTY(double SigmaY READ sigmaY WRITE sigmaY)
  Q_PROPERTY(BorderType BorderType READ borderType WRITE borderType)

public:

  Q_INVOKABLE explicit GaussianBlurFilter(QObject* parent = nullptr);

  virtual ~GaussianBlurFilter() {}

  int kernelSizeX() const { return _kernelSizeX; }
  void kernelSizeX(int kernelSizeX) {
    if (kernelSizeX < 1) kernelSizeX = 3;
    _kernelSizeX = 1 + (kernelSizeX / 2) * 2;
  }

  int kernelSizeY() const { return _kernelSizeY; }
  void kernelSizeY(int kernelSizeY) {
    if (kernelSizeY < 1) kernelSizeY = 3;
    _kernelSizeY = 1 + (kernelSizeY / 2) * 2;
  }

  double sigmaX() const { return _sigmaX; }
  void sigmaX(double sigmaX) { _sigmaX = sigmaX; }

  double sigmaY() const { return _sigmaY; }
  void sigmaY(double sigmaY) { _sigmaY = sigmaY; }



  BorderType borderType() const { return _borderType; }
  void borderType(BorderType borderType) { _borderType = borderType; }

public slots:

  void in(const MatEvent& input);

signals:

  void out(const MatEvent& output);

protected:

private:

  int _kernelSizeX;
  int _kernelSizeY;

  double _sigmaX;
  double _sigmaY;

  BorderType _borderType;

};

#endif // GAUSSIANBLURFILTER_H
