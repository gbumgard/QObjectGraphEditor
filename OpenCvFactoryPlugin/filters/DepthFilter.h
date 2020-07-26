#ifndef DEPTHFILTER_H
#define DEPTHFILTER_H

#include <QObject>

#include <opencv2/core.hpp>

class DepthFilter : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Depth Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(qreal fastGain READ fastGain WRITE fastGain)
  Q_PROPERTY(qreal slowGain READ slowGain WRITE slowGain)
  Q_PROPERTY(qreal hysteresis READ hysteresis WRITE hysteresis)

public:

  Q_INVOKABLE explicit DepthFilter(QObject* parent = nullptr);

  virtual ~DepthFilter() {}

  qreal slowGain() const { return _slowGain; }

  qreal fastGain() const { return _fastGain; }

  qreal hysteresis() const { return _hysteresis; }

public slots:

  void in(const cv::Mat& mat);

  void slowGain(qreal gain);

  void fastGain(qreal gain);

  void hysteresis(qreal level);

signals:

  void out(const cv::Mat& mat);
  void var(const cv::Mat& mat);

protected:

  cv::Size _size;

  qreal _fastGain;
  qreal _slowGain;
  qreal _hysteresis;

  cv::Mat _fastAverage;
  cv::Mat _slowAverage;
  cv::Mat _slowVariance;

  cv::Mat _filteredSample;

};

#endif // DEPTHFILTER_H
