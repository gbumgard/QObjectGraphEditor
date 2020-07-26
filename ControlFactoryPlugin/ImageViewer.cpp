#include "ImageViewer.h"
#include "ControlFactoryPlugin.h"

#include <QPainter>

#include <QDebug>

REGISTER_CLASS(ImageViewer)

ImageViewer::ImageViewer(QWidget *parent)
  : QWidget(parent)
{
  setProperty("methodOffset",QWidget::staticMetaObject.methodOffset());
  setAutoFillBackground(false);
  setAttribute(Qt::WA_NoSystemBackground, true);
  setSizePolicy(QSizePolicy::MinimumExpanding,QSizePolicy::MinimumExpanding);
}

void ImageViewer::in(const QImage& image) {
  _image = image;
  if (image.size() != _imageSize) {
    _imageSize = _image.size();
    setMinimumSize(_imageSize.width(),_imageSize.height());
    adjustSize();
  }
  repaint();
}

void ImageViewer::paintEvent(QPaintEvent* /*event*/)
{
  if (!_image.isNull()) {
    QPainter painter(this);
    painter.drawImage(QPoint(0,0),_image.scaled(this->size(),Qt::KeepAspectRatio));
    painter.end();
  }
}
