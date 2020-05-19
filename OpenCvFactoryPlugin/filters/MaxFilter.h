#ifndef MaxFilter_H
#define MaxFilter_H

#include <QObject>

#include <opencv2/core.hpp>

class MaxFilter : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Max Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(qreal maxValue READ maxValue WRITE maxValue)

public:

  Q_INVOKABLE explicit MaxFilter(QObject* parent = nullptr);

  virtual ~MaxFilter() {}

  qreal maxValue() const { return _maxValue; }

public slots:

  void in(const cv::Mat& mat);

  void maxValue(qreal maxValue);

signals:

  void out(const cv::Mat& mat);

protected:

  qreal _maxValue;

  cv::Mat _maxResult;

};

#endif // MaxFilter_H
