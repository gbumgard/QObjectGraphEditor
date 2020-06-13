#ifndef RunningStatistics_H
#define RunningStatistics_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>

#include <opencv2/core.hpp>

class RunningStatistics : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Running Statistics")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(qreal min READ min WRITE min)
  Q_PROPERTY(qreal max READ max WRITE max)
  Q_PROPERTY(bool reset READ reset WRITE reset)

public:

  Q_INVOKABLE explicit RunningStatistics(QObject* parent = nullptr);

  virtual ~RunningStatistics();

  qreal min() const { return _min; }

  qreal max() const { return _max; }

  bool reset() const { return false; }

public slots:

  void in(const TaggedMat& taggedMat);

  void min(qreal min) { _min  = min; clear(); }

  void max(qreal max) { _max  = max; clear(); }

  void reset(bool) { clear(); }

signals:

  void mean(const TaggedMat& mat);

  void var(const TaggedMat& mat);

  void std(const TaggedMat& mat);

  void update(const TaggedMat& mat);

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
