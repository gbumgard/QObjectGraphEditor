#ifndef STATFILTER_H
#define STATFILTER_H

#include "AbstractOpenCvObject.h"
#include <QObject>

#include <opencv2/core.hpp>

class StatFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Statistical Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(bool enable READ enable WRITE enable NOTIFY enableChanged)

public:

  Q_INVOKABLE explicit StatFilter(QObject* parent = nullptr);

  virtual ~StatFilter();

  bool enable() const { return _enable; }

  void enable(bool enable) {
    if (enable && !_enable) clear();
    _enable = enable;
  }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& dstEvent);

signals:

  QVARIANT_PAYLOAD(MatEvent) void mean(const QVariant& dstEvent);

  QVARIANT_PAYLOAD(MatEvent) void variance(const QVariant& dstEvent);

  QVARIANT_PAYLOAD(MatEvent) void filtered(const QVariant& dstEvent);

  void enableChanged(bool);

protected:

  void clear();

  cv::Size _size;

  bool _enable;
  uint32_t* _count;
  cv::Mat _aggregateMean;
  cv::Mat _aggregateMeanSquared;
  cv::Mat _variance;
  cv::Mat _filtered;


};

#endif // STATFILTER_H
