#ifndef DepthFilterV2_H
#define DepthFilterV2_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>

#include <opencv2/core.hpp>

class DepthFilterV2 : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Depth Filter V2")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(qreal range READ range WRITE range)
  Q_PROPERTY(qreal fastGain READ fastGain WRITE fastGain)
  Q_PROPERTY(qreal slowGain READ slowGain WRITE slowGain)
  Q_PROPERTY(qreal min READ min WRITE min)
  Q_PROPERTY(qreal max READ max WRITE max)

public:

  Q_INVOKABLE explicit DepthFilterV2(QObject* parent = nullptr);

  virtual ~DepthFilterV2() {}

  qreal range() const { return _range; }
  void range(qreal range);

  qreal slowGain() const { return _slowGain; }
  void slowGain(qreal gain);

  qreal fastGain() const { return _fastGain; }
  void fastGain(qreal gain);

  qreal min() const { return _min; }
  qreal max() const { return _max; }

  void reset();

public slots:

  void in(const MatEvent& mat);

  void min(qreal min) {
    _min = min;
    reset();
  }

  void max(qreal max) {
    _max = max;
    reset();
  }

signals:

  void out(const MatEvent& mat);
  void slow(const MatEvent& mat);
  void slowVar(const MatEvent& mat);
  void slowStd(const MatEvent& mat);
  void fast(const MatEvent& mat);
  void fastVar(const MatEvent& mat);
  void fastStd(const MatEvent& mat);
  void alpha(const MatEvent& mat);
  void msk(const MatEvent& mat);

protected:

  qreal _range;
  qreal _fastGain;
  qreal _slowGain;
  qreal _min;
  qreal _max;

  cv::Mat _fastAverage;
  cv::Mat _fastVariance;
  cv::Mat _slowAverage;
  cv::Mat _slowVariance;
  cv::Mat _average;

};

#endif // DepthFilterV2_H
