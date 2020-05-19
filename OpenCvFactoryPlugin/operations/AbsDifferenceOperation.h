#ifndef ABSDIFFERENCEOPERATION_H
#define ABSDIFFERENCEOPERATION_H

#include <QObject>

#include <opencv2/core.hpp>

class AbsDifferenceOperations : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Abs Difference")
  Q_CLASSINFO("directory","OpenCV/Operations")

public:

  Q_INVOKABLE explicit AbsDifferenceOperations(QObject* parent = nullptr);

  virtual ~AbsDifferenceOperations() {}

public slots:

  void alpha(const cv::Mat& alpha);
  void beta(const cv::Mat& beta);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _alphaMat;
  cv::Mat _betaMat;

};

#endif // ABSDIFFERENCEOPERATION_H
