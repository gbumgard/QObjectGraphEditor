#ifndef RUNNINGSTATSUTILITY_H
#define RUNNINGSTATSUTILITY_H

#include <QObject>

#include <opencv2/core.hpp>

class RunningStatsUtility : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Statistics")
  Q_CLASSINFO("directory","OpenCV/Utility")

  Q_PROPERTY(bool reset READ reset WRITE reset)
  Q_PROPERTY(qreal resetThreshold READ resetThreshold WRITE resetThreshold)
  Q_PROPERTY(int minimumSamples READ minimumSamples WRITE minimumSamples)

public:

  Q_INVOKABLE explicit RunningStatsUtility(QObject* parent = nullptr);

  virtual ~RunningStatsUtility() {}

  bool reset() const { return true; }

  void reset(bool) { clear(); }

  int minimumSamples() const { return _minimumSamples; }
  void minimumSamples(int minimumSamples) { _minimumSamples = minimumSamples; }


  qreal resetThreshold() const { return _resetThreshold; }
  void resetThreshold(qreal resetThreshold) { _resetThreshold = resetThreshold; }

public slots:

  void in(const cv::Mat& mat);

signals:

  void mean(const cv::Mat& mat);

  void variance(const cv::Mat& mat);

  void sd(const cv::Mat& mat);

protected:

  void clear();

  void push(double x);

  int _minimumSamples;
  qreal _resetThreshold;

  cv::Size _size;

  cv::Mat _count;
  cv::Mat _oldRunningMean;
  cv::Mat _newRunningMean;
  cv::Mat _oldRunningSigma;
  cv::Mat _newRunningSigma;
  cv::Mat _variance;
  cv::Mat _sigma;

};

#endif // RUNNINGSTATSUTILITY_H
