#ifndef MedianFilter3D_H
#define MedianFilter3D_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MedianFilter3D : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","3D Median Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

private:

  Q_PROPERTY(AperatureSize aperatureSize READ aperatureSize WRITE setAperatureSize)

public:

  enum AperatureSize {
    Kernel3x3x3 = 3,
    Kernel5x5x5 = 5
  };

  Q_ENUM(AperatureSize)

  Q_INVOKABLE explicit MedianFilter3D(QObject* parent = nullptr);

  virtual ~MedianFilter3D() {}

  AperatureSize aperatureSize() const { return _aperatureSize; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& dstEvent);

  void setAperatureSize(AperatureSize aperatureSize);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

protected:

  template <typename T>
  void computeMedian(cv::Mat& mat) const {
    int center = _aperatureSize / 2;
    std::vector<T> values(_aperatureSize);
    for (int channel = 0; channel < mat.channels(); channel++) {
      for (int row = 0; row < mat.rows; row++) {
        for (int col = 0; col < mat.cols; col++) {
          for (size_t frame = 0; frame < _aperatureSize; frame++) {
            values[frame] = _frameBuffer.at(frame).at<T>(row,col);
          }
          std::nth_element(values.begin(), values.begin()+center, values.end());
          mat.at<T>(row,col) = values[center];
        }
      }
    }
  }

private:

  AperatureSize _aperatureSize;
  std::vector<cv::Mat> _frameBuffer;

};

#endif // MedianFilter3D_H
