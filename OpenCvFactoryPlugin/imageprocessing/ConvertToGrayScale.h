#ifndef CONVERTTOGRAYSCALE_H
#define CONVERTTOGRAYSCALE_H

#include <QObject>
#include <opencv2/imgproc.hpp>

class ConvertToGrayScale : public QObject
{

  Q_OBJECT

public:

private:

  Q_CLASSINFO("class-alias","Convert To Grayscale")
  Q_CLASSINFO("directory","OpenCV/Image Processing")


public:

  Q_INVOKABLE ConvertToGrayScale(QObject* parent = nullptr);

signals:

  void out(const cv::Mat& mat);

public slots:

  void in(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _input;

};

#endif // CONVERTTOGRAYSCALE_H
