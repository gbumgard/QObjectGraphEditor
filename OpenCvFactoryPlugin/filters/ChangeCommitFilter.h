#ifndef CHANGECOMMITFILTER_H
#define CHANGECOMMITFILTER_H

#include <QObject>
#include <opencv2/core.hpp>

class ChangeCommitFilter : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Change Commit Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(int startCount READ startCount WRITE startCount)
  Q_PROPERTY(qreal startThreshold READ startThreshold WRITE startThreshold)

public:

  Q_INVOKABLE explicit ChangeCommitFilter(QObject* parent = nullptr);

  virtual ~ChangeCommitFilter() {}

  short startCount() const { return _startCount; }

  qreal startThreshold() const { return _startThreshold; }

public slots:

  void in(const cv::Mat& mat);

  void startCount(int startCount);

  void startThreshold(qreal startThreshold);

signals:

  void out(const cv::Mat& mat);

  void countingMask(const cv::Mat& mat);
  void absDifference(const cv::Mat& mat);
  void startThresholdMask(const cv::Mat& mat);
  void startCountdownMask(const cv::Mat& mat);
  void currentCount(const cv::Mat& mat);
  void filterMask(const cv::Mat& mat);
  void countDecrementMask(const cv::Mat& mat);

protected:

  cv::Size _size;

  int _startCount;
  qreal _startThreshold;

  cv::Mat _currentCount;

  cv::Mat _filteredInput;

};

#endif // CHANGECOMMITFILTER_H
