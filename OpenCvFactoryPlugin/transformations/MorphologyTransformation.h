#ifndef MORPHOLOGYTRANSFORMATION_H
#define MORPHOLOGYTRANSFORMATION_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include <opencv2/imgproc.hpp>

class MorphologyTransformation : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Morphology Transformation")
  Q_CLASSINFO("directory","OpenCV/Transformations")

public:

  enum KernelSize {
    KERNEL_3X3 = 3,
    KERNEL_5x5 = 5,
    KERNEL_7x7 = 7
  };

  Q_ENUM(KernelSize)

  enum Operation {
    OPEN = cv::MORPH_OPEN,
    CLOSE = cv::MORPH_CLOSE,
    GRADIENT = cv::MORPH_GRADIENT,
    TOPHAT = cv::MORPH_TOPHAT,
    BLACKHAT = cv::MORPH_BLACKHAT,
    HITMISS = cv::MORPH_HITMISS
  };

  Q_ENUM(Operation)

  enum Shape {
    RECT = cv::MORPH_RECT,
    ELLIPSE = cv::MORPH_ELLIPSE,
    CROSS = cv::MORPH_CROSS
  };

  Q_ENUM(Shape)

private:

  Q_PROPERTY(Operation operation READ operation WRITE operation)
  Q_PROPERTY(Shape shape READ shape WRITE shape)
  Q_PROPERTY(KernelSize kernelSize READ kernelSize WRITE kernelSize)
  Q_PROPERTY(int iterations READ iterations WRITE iterations)

public:


  Q_INVOKABLE explicit MorphologyTransformation(QObject* parent = nullptr);

  virtual ~MorphologyTransformation() {}

  Operation operation() const { return _operation; }
  void operation(Operation operation);

  Shape shape() const { return _shape; }
  void shape(Shape shape);

  KernelSize kernelSize() const { return _kernelSize; }
  void kernelSize(KernelSize kernelSize);

  int iterations() const { return _iterations; }
  void iterations(int iterations);

public slots:

  void in(const MatEvent& input);

signals:

  void out(const MatEvent& output);

protected:

private:

  Operation _operation;
  Shape _shape;
  KernelSize _kernelSize;
  int _iterations;

};

#endif // MORPHOLOGYTRANSFORMATION_H
