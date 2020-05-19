#ifndef RUNNINGAVERAGEFILTER_H
#define RUNNINGAVERAGEFILTER_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class RunningAverageFilter : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Running Average Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

  Q_PROPERTY(double weight READ weight WRITE weight)

public:

  Q_INVOKABLE explicit RunningAverageFilter(QObject* parent = nullptr);

  virtual ~RunningAverageFilter() {}

  double weight() const { return _weight; }

public slots:

  void in(const cv::Mat& mat);

  void weight(double weight);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  double _weight;

  cv::Mat _average;
  cv::Mat _input;

};

#endif // RUNNINGAVERAGEFILTER_H
