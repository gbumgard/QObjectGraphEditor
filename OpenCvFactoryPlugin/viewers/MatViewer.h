#ifndef TaggedMatViewer_H
#define TaggedMatViewer_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include <QWidget>
#include <QImage>
#include <opencv2/core.hpp>

class MatViewer : public QWidget
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Mat Viewer")
  Q_CLASSINFO("directory","OpenCV/Output")

  Q_PROPERTY(QString caption READ caption WRITE setCaption)
  Q_PROPERTY(bool SwapRedBlue READ swapRedBlue WRITE swapRedBlue)

public:

  Q_INVOKABLE explicit MatViewer(QWidget* parent = nullptr);

  QString caption() const {
    return QObject::objectName();
  }

  void setCaption(const QString& caption) {
    QObject::setObjectName(caption);
  }

  QImage image() const { return _image; }

  bool swapRedBlue() const { return _swapRedBlue; }
  void swapRedBlue(bool swapRedBlue) { _swapRedBlue = swapRedBlue; }

  QSize sizeHint() const {
    return _image.size();
  }

public slots:

  void in(const TaggedMat& taggedImage);

protected:

  void paintEvent(QPaintEvent *event);

private:

  bool _swapRedBlue;
  QImage _image;
  cv::Size _imageSize;

};

#endif // TaggedMatViewer_H
