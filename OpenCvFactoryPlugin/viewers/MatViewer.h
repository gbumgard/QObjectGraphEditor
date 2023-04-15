#ifndef MatEventViewer_H
#define MatEventViewer_H

#include <QObject>
#include <QWidget>
#include <QImage>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MatViewer : public QWidget
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Mat Viewer")
  Q_CLASSINFO("directory","OpenCV/Output")
  Q_CLASSINFO("slots","in(QVariant)")
  Q_CLASSINFO("signals","")

  Q_PROPERTY(QString caption READ caption WRITE caption STORED true FINAL NOTIFY captionChanged)
  Q_PROPERTY(bool swapRedBlue READ swapRedBlue WRITE setSwapRedBlue STORED true FINAL NOTIFY swapRedBlueChanged )

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

  QSize sizeHint() const {
    return _image.size();
  }

public slots:

  QVARIANT_PAYLOAD(MatEvent)
  QVARIANT_PAYLOAD(cv::Mat)
  QVARIANT_PAYLOAD(QImage)
  void in(const QVariant& variant);

  void setSwapRedBlue(bool swapRedBlue) {
    _swapRedBlue = swapRedBlue;
  }


signals:

  void captionChanged(const QString&);
  void swapRedBlueChanged(bool);

protected:

  void paintEvent(QPaintEvent *event);

private:

  bool _swapRedBlue;
  QImage _image;
  cv::Size _imageSize;

};

#endif // MatEventViewer_H
