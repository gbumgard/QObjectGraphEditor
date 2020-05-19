#ifndef MATVIEWER_H
#define MATVIEWER_H

#include <QObject>
#include <QWidget>
#include <QImage>
#include <opencv2/core.hpp>

class MatViewer : public QWidget
{
  Q_OBJECT

  Q_CLASSINFO("datasink","true")
  Q_CLASSINFO("setImage(Mat)","in")
  Q_CLASSINFO("class-alias","Mat Viewer")
  Q_CLASSINFO("directory","OpenCV/Output")

public:

  Q_INVOKABLE explicit MatViewer(QWidget* parent = nullptr);

  QImage image() const { return _image; }

  QSize sizeHint() const {
    return _image.size();
  }

public slots:

  void in(const cv::Mat& image);

protected:

  void paintEvent(QPaintEvent *event);

private:

  QImage _image;
  cv::Size _imageSize;

};

#endif // MATVIEWER_H
