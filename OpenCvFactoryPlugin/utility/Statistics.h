#ifndef STATISTICS_H
#define STATISTICS_H

#include <QObject>

#include <opencv2/core.hpp>

class Statistics : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mat Statistics")
  Q_CLASSINFO("directory","OpenCV/Utility")

  Q_PROPERTY(bool enable READ enable WRITE setEnable)
  Q_PROPERTY(int count READ count)

public:

  Q_INVOKABLE explicit Statistics(QObject* parent = nullptr);

  virtual ~Statistics() {}

  bool enable() const { return _enable; }

  void setEnable(bool enable) {
    if (enable && !_enable) clear();
    _enable = enable;
  }

  int count() const { return _count; }

public slots:

  void in(const cv::Mat& mat);

signals:

  void mean(const cv::Mat& mat);

  void variance(const cv::Mat& mat);

protected:

  void clear();

  cv::Size _size;

  bool _enable;
  int _count;
  cv::Mat _aggregateMean;
  cv::Mat _aggregateMeanSquared;

};

#endif // STATISTICS_H
