#ifndef TEMPORALMEDIANFILTER_H
#define TEMPORALMEDIANFILTER_H

#include <QObject>
#include <opencv2/core.hpp>
#include <vector>
#include <QMutex>
#include <QVariant>
#include <QFuture>

#include "MatEvent.h"
#include "AbstractOpenCvObject.h"

class TemporalMedianFilter : public AbstractOpenCvObject
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

  void aperatureSize(int aperatureSize);

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& event);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& event);

protected:

private:

  size_t _aperatureSize;
  std::vector<cv::Mat> _frameBuffer;
  QFuture<cv::Mat> _task;
};

#endif // TEMPORALMEDIANFILTER_H
