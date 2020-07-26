#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QObject>
#include <QWidget>
#include <QImage>

class ImageViewer : public QWidget
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Image Viewer")
  Q_CLASSINFO("directory","Qt/Viewers")

  Q_PROPERTY(QString caption READ caption WRITE setCaption)

public:

  Q_INVOKABLE explicit ImageViewer(QWidget* parent = nullptr);

  QImage image() const { return _image; }

  QSize sizeHint() const {
    return _image.size();
  }

  QString caption() const {
    return QObject::objectName();
  }

  void setCaption(const QString& caption) {
    QObject::setObjectName(caption);
  }

public slots:

  void in(const QImage& image);

protected:

  void paintEvent(QPaintEvent *event);

private:

  QImage _image;
  QSize _imageSize;

};

#endif // IMAGEVIEWER_H
