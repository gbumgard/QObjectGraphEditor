#ifndef ThreadDecoupler_H
#define ThreadDecoupler_H

#include <QObject>
#include <QMetaType>
#include <QPoint>
#include "ThreadedObject.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

Q_DECLARE_METATYPE(std::vector<std::vector<cv::Point>>)

class ThreadDecoupler : public ThreadedObject
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Thread Decoupler")
  Q_CLASSINFO("directory","OpenCV/Utility")

public:

  Q_INVOKABLE explicit ThreadDecoupler(QObject *parent = nullptr);

  ~ThreadDecoupler();

signals:

  void out(const cv::Mat& mat);

public slots:

  void in(const cv::Mat& mat);

protected:

  void update() override;

private:

  cv::Mat _nextInput;

};

#endif // ThreadDecoupler_H
