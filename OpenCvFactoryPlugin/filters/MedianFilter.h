#ifndef MEDIANFILTER_H
#define MEDIANFILTER_H

#include <QObject>
#include <opencv2/core.hpp>

class MedianFilter : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Median Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

private:

  Q_PROPERTY(int aperatureSize READ aperatureSize WRITE setAperatureSize)

public:

  Q_INVOKABLE explicit MedianFilter(QObject* parent = nullptr);

  virtual ~MedianFilter() {}

  int aperatureSize() const { return _aperatureSize; }

public slots:

  void in(const cv::Mat& mat);

  void setAperatureSize(int aperatureSize);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  int _aperatureSize;

  cv::Mat _input;

};

#endif // MEDIANFILTER_H
