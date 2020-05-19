#ifndef TEMPORALMEDIANFILTER_H
#define TEMPORALMEDIANFILTER_H

#include <QObject>
#include "ThreadedObject.h"
#include <opencv2/core.hpp>
#include <vector>
#include <QMutex>

class TemporalMedianFilter : public ThreadedObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Temporal Median Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

private:

  Q_PROPERTY(int aperatureSize READ aperatureSize WRITE aperatureSize)

public:

  Q_INVOKABLE explicit TemporalMedianFilter(QObject* parent = nullptr);

  virtual ~TemporalMedianFilter() {}

  int aperatureSize() const { return _aperatureSize; }

public slots:

  void in(const cv::Mat& mat);

  void aperatureSize(int aperatureSize);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  QMutex _mutex;
  int _aperatureSize;
  int _width;
  int _height;
  std::vector<cv::Mat> _inputBuffer;
  std::vector<cv::Mat> _processingBuffer;

};

#endif // TEMPORALMEDIANFILTER_H
