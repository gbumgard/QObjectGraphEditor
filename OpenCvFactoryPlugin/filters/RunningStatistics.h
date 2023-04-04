#ifndef RunningStatistics_H
#define RunningStatistics_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class RunningStatistics : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Running Statistics")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(qreal min READ min WRITE min NOTIFY minChanged)
  Q_PROPERTY(qreal max READ max WRITE max NOTIFY maxChanged)
  Q_PROPERTY(bool reset READ reset WRITE reset NOTIFY resetChanged)

public:

  Q_INVOKABLE explicit RunningStatistics(QObject* parent = nullptr);

  virtual ~RunningStatistics();

  qreal min() const { return _min; }

  qreal max() const { return _max; }

  bool reset() const { return false; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

  void min(qreal min) { _min  = min; clear(); }

  void max(qreal max) { _max  = max; clear(); }

  void reset(bool) { clear(); }

signals:

  QVARIANT_PAYLOAD(MatEvent) void mean(const QVariant& dstEvent);

  QVARIANT_PAYLOAD(MatEvent) void var(const QVariant& dstEvent);

  QVARIANT_PAYLOAD(MatEvent) void std(const QVariant& dstEvent);

  QVARIANT_PAYLOAD(MatEvent) void update(const QVariant& dstEvent);

  void minChanged(qreal);
  void maxChanged(qreal);
  void resetChanged(bool);

protected:

  void clear();

private:

  cv::Mat _count;
  cv::Mat _oldMean;
  cv::Mat _newMean;
  cv::Mat _oldVarianceSquared;
  cv::Mat _newVarianceSquared;
  qreal _min;
  qreal _max;
};

#endif // RunningStatistics_H
