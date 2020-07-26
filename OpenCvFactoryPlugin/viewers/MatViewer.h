#ifndef MatEventViewer_H
#define MatEventViewer_H

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
  Q_CLASSINFO("slot-order","src(QVariant)")
  Q_CLASSINFO("signal-order","NONE")

  Q_PROPERTY(QString caption READ caption WRITE caption STORED true FINAL)
  Q_PROPERTY(bool SwapRedBlue READ swapRedBlue WRITE swapRedBlue STORED true FINAL)

public:

  Q_INVOKABLE explicit MatViewer(QObject* parent = nullptr);

  virtual ~MatViewer();

  QString caption() const {
    return QObject::objectName();
  }

  void caption(const QString& caption) {
    QObject::setObjectName(caption);
  }

  bool swapRedBlue() const {
    return _swapRedBlue;
  }

  void swapRedBlue(bool swapRedBlue) {
    _swapRedBlue = swapRedBlue;
  }

  QSize sizeHint() const {
    return _image.size();
  }

public slots:

  QVARIANT_PAYLOAD(MatEvent)
  QVARIANT_PAYLOAD(cv::Mat)
  QVARIANT_PAYLOAD(QImage)
  void src(const QVariant& variant);

signals:


protected:

  void paintEvent(QPaintEvent *event);

private:

  bool _swapRedBlue;
  QImage _image;
  cv::Size _imageSize;

};

#endif // MatEventViewer_H
