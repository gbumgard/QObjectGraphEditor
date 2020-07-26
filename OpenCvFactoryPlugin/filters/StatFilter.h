#ifndef STATFILTER_H
#define STATFILTER_H

#include <QObject>

#include <opencv2/core.hpp>

class StatFilter : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Statistical Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(bool enable READ enable WRITE enable)

public:

  Q_INVOKABLE explicit StatFilter(QObject* parent = nullptr);

  virtual ~StatFilter();

  bool enable() const { return _enable; }

  void enable(bool enable) {
    if (enable && !_enable) clear();
    _enable = enable;
  }

public slots:

  void in(const cv::Mat& mat);

signals:

  void mean(const cv::Mat& mat);

  void variance(const cv::Mat& mat);

  void filtered(const cv::Mat& mat);

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
