#include "ObjectView.h"

#include <QStyle>
#include <QIcon>
#include <QMimeData>
#include <QMetaObject>
#include <QMetaProperty>
#include <QGraphicsSceneEvent>
#include <QGraphicsItem>
#include <QGraphicsSceneDragDropEvent>
#include <QOpenGLWidget>

#include <QDebug>
#include <math.h>

#include "ObjectGraphNode.h"

ObjectView::ObjectView(QWidget* parent)
  : QGraphicsView(parent)
  , _recenterButton(new QToolButton(this))
{
  connect(_recenterButton,&QToolButton::pressed,this,&ObjectView::onRecenterPressed);
  connect(this,&ObjectView::rubberBandChanged,this,&ObjectView::onRubberBandChanged);
  _recenterButton->setIcon(_recenterButton->style()->standardIcon(QStyle::SP_TitleBarNormalButton));
  setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
  //QOpenGLWidget* glWidget = new QOpenGLWidget(this);
  //QSurfaceFormat format;
  //format.setSamples(4);
  //glWidget->setFormat(format);
  //setViewport(glWidget);
  //setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  //this->update();

}

ObjectView::ObjectView(QGraphicsScene* scene, QWidget* parent)
  : QGraphicsView(scene,parent)
{

}

ObjectView::~ObjectView() {

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
      qDebug() << metaObj->className() << property.name() << property.typeName();
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

void ObjectView::onRubberBandChanged(QRect rubberBandRect, QPointF fromScenePoint, QPointF toScenePoint) {
  qDebug() << Q_FUNC_INFO << rubberBandRect << fromScenePoint << toScenePoint;
  if (!rubberBandRect.isNull()) {
    _selectionArea = rubberBandRect;
  }
}

void ObjectView::contextMenuEvent(QContextMenuEvent *event) {
  qDebug() << Q_FUNC_INFO << event;
  QGraphicsView::contextMenuEvent(event);
}

void ObjectView::mousePressEvent(QMouseEvent *event) {

  qDebug() << "\nPRESS" << Q_FUNC_INFO << event->type() << this->mapToScene(event->pos());

  if (event->button() == Qt::LeftButton && event->modifiers().testFlag(Qt::ControlModifier)) {
    setDragMode(QGraphicsView::RubberBandDrag);
  }

  QGraphicsView::mousePressEvent(event);
}

void ObjectView::mouseReleaseEvent(QMouseEvent *event) {

  qDebug() << "\nRELEASE" << Q_FUNC_INFO << event->type() << this->mapToScene(event->pos());

  QGraphicsView::mouseReleaseEvent(event);

  if (event->button() == Qt::LeftButton) {

    if (event->modifiers().testFlag(Qt::ControlModifier)) {

      // QGraphicsView will have selected all items in the rubber-band selection box.
      // We want to deselect the items if the SHIFT key was also pressed along with CTRL.

      if (!_selectionArea.isNull()) {

        bool select = !event->modifiers().testFlag(Qt::ShiftModifier);

        QList<QGraphicsItem*> enclosedItems = items(_selectionArea,Qt::ContainsItemShape);

        for (auto item : enclosedItems) {

          ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);

          if (node) {
            qDebug() << "node" << node->uuid() << "select is" << select;
            node->setSelected(select);
          }

        }
      }
    }
  }

  // The drag mode must be restored AFTER the mouse event has been processed
  // in the QGraphicsView::mouseReleaseEvent method. If not, the items within
  // the selection box will be deselected when the mode is changed.

  setDragMode(QGraphicsView::ScrollHandDrag);

}

