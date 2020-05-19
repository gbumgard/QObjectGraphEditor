#ifndef TEMPORALDEPTHDENOISEFILTER_H
#define TEMPORALDEPTHDENOISEFILTER_H

#include <QObject>

#include <deque>

#include <opencv2/core.hpp>

class KinectV1DepthFilter : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Kinect V1 Depth Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(double timeInterval READ timeInterval WRITE setTimeInterval)
  Q_PROPERTY(qreal gain READ gain WRITE gain)
  Q_PROPERTY(double minimumVelocity READ minimumVelocity WRITE setMinimumVelocity)
  Q_PROPERTY(double maximumVelocity READ maximumVelocity WRITE setMaximumVelocity)

public:

  Q_INVOKABLE explicit KinectV1DepthFilter(QObject* parent = nullptr);

  virtual ~KinectV1DepthFilter() {}

  double timeInterval() const { return _timeIntervalMs; }
  void setTimeInterval(double timeIntervalMs);

  double gain() const { return _minimumGain; }

  double minimumVelocity() const { return _minimumVelocity; }
  void setMinimumVelocity(double velocity);

  double maximumVelocity() const { return _maximumVelocity; }
  void setMaximumVelocity(double velocity);

public slots:

  void in(const cv::Mat& mat);

  void gain(qreal value);

signals:

  void out(const cv::Mat& mat);

protected:

  struct Sample {
    qint64 epochTimestampMs;
    cv::Mat depthFrame;
  };

private:

  qint64 _timeIntervalMs;
  qreal _minimumGain;

  double _minimumVelocity;
  double _maximumVelocity;

  std::deque<Sample> _rawSampleQueue;
  Sample _filteredSample;

};


#endif // TEMPORALDEPTHDENOISEFILTER_H
