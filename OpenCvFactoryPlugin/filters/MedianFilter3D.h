#ifndef MedianFilter3D_H
#define MedianFilter3D_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>
#include <thread>
#include <future>
#include <algorithm>

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

private slots:

  void start();

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

  void finished();

protected:

  template <typename T>
  static T computeMedian(std::vector<T> values) {
      std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
      return values[values.size()/2];
  }

  template <typename T>
  static void computeMedian(std::vector<cv::Mat> frameBuffer, cv::Mat& output) {
      for (int row = 0; row < frameBuffer[0].rows; row++) {
        for (int col = 0; col < frameBuffer[0].cols; col++) {
          std::vector<T> values;
          for (auto& frame : frameBuffer) {
            values.push_back(frame.at<T>(row,col));
          }
          output.at<T>(row,col) = computeMedian<T>(values);
        }
      }
  }

private:

  AperatureSize _aperatureSize;
  std::vector<cv::Mat> _frameBuffer;
  std::future<void> _taskFuture;
};

#endif // MedianFilter3D_H
