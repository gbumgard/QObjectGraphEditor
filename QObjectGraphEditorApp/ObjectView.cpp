#include "ObjectView.h"
#include <QStyle>
#include <QIcon>
#include <QMimeData>
#include <QMetaObject>
#include <QMetaProperty>
#include <QGraphicsSceneEvent>
#include <QGraphicsSceneDragDropEvent>

#include <QDebug>
#include <math.h>

ObjectView::ObjectView(QWidget* parent)
  : QGraphicsView(parent)
  , _numScheduledScalings(0)
  , _scaleSteps(0.0)
  , _recenterButton(new QToolButton(this))
{
  connect(_recenterButton,&QToolButton::pressed,this,&ObjectView::onRecenterPressed);
  _recenterButton->setIcon(_recenterButton->style()->standardIcon(QStyle::SP_TitleBarNormalButton));
  setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
}

ObjectView::ObjectView(QGraphicsScene* scene, QWidget* parent)
  : QGraphicsView(scene,parent)
{

}

bool ObjectView::load(const QString& fileName) {
  QFile file(fileName);
  file.open(QIODevice::ReadOnly);
  QDataStream in(&file);
  return read(in);
}

bool ObjectView::save(const QString& fileName) const {
  QFile file(fileName);
  file.open(QIODevice::WriteOnly);
  QDataStream out(&file);
  return write(out);
}

bool ObjectView::read(QDataStream &in) {

  QMap<int,QVariant> viewProperties;
  in >> viewProperties;

  const QMetaObject* metaObj = metaObject();
  for (auto index : viewProperties.keys()) {
    metaObj->property(index).write(this,viewProperties[index]);
  }
  return true;
}

bool ObjectView::write(QDataStream &out) const {
  QMap<int,QVariant> viewProperties;
  const QMetaObject* metaObj = metaObject();
  for (int i = 0; i < metaObj->propertyCount(); i++) {
    QMetaProperty property = metaObj->property(i);
    if (property.isValid() &&
        property.isReadable() &&
        property.isWritable() &&
        property.isStored()) {
      viewProperties.insert(i,property.read(this));
    }
  }
  out << viewProperties;
  return true;
}


void ObjectView::onRecenterPressed() {
  fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void ObjectView::resizeEvent(QResizeEvent *event) {

  _recenterButton->move(10,10);
  QGraphicsView::resizeEvent(event);
}

void ObjectView::wheelEvent(QWheelEvent* event)
{
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  QPoint numDegrees = event->angleDelta();
  static const double stepScale = 1.15;

  if (!numDegrees.isNull()) {
    int numSteps = numDegrees.ry() / 120;
    double scaleFactor = 0;
    if (numSteps < 0) {
      scaleFactor = pow(1.0/stepScale,-numSteps);
    }
    else {
      scaleFactor = pow(stepScale,numSteps);
    }
    scale(scaleFactor,scaleFactor);
  }
}

void ObjectView::mouseMoveEvent(QMouseEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type() << this->mapToScene(event->pos());
  QGraphicsView::mouseMoveEvent(event);
}

void ObjectView::mousePressEvent(QMouseEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type() << this->mapToScene(event->pos());
  QGraphicsView::mousePressEvent(event);
}

void ObjectView::mouseReleaseEvent(QMouseEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type() << this->mapToScene(event->pos());
  QGraphicsView::mouseReleaseEvent(event);
}

void ObjectView::dragEnterEvent(QDragEnterEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type() << this->mapToScene(event->pos());
  QGraphicsView::dragEnterEvent(event);
}

void ObjectView::dragLeaveEvent(QDragLeaveEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type();
  QGraphicsView::dragLeaveEvent(event);
}

void ObjectView::dropEvent(QDropEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type() << mapToScene(event->pos()) << event->mimeData()->formats();
  QGraphicsView::dropEvent(event);
}

void ObjectView::dragMoveEvent(QDragMoveEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->type() <<  mapToScene(event->pos()) << event->mimeData()->formats();
  QGraphicsView::dragMoveEvent(event);
}

