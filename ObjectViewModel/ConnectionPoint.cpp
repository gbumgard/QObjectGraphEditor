#include "ConnectionPoint.h"
#include "ObjectGraph.h"

#include <QApplication>
#include <QGraphicsSceneDragDropEvent>
#include <QDrag>
#include <QWidget>
#include <QMimeData>
#include "ObjectGraphNode.h"


#include <QDebug>

ConnectionPoint::ConnectionPoint(const QMetaMethod& metaMethod,
                                 QGraphicsItem* parent)
  : QGraphicsEllipseItem(-RADIUS,-RADIUS,2*RADIUS,2*RADIUS,parent)
  , _metaMethod(metaMethod)
{
  setAcceptDrops(true);
  setAcceptHoverEvents(true);
  setPen(QPen(QColor("white"),2));
  setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
  ObjectGraphNode* objectNode = dynamic_cast<ObjectGraphNode*>(parent);
  QObject* object = objectNode->object();
  if (object) {
    if (metaMethod.isValid()) {
      setToolTip(metaMethod.methodSignature());
      setObjectName(object->objectName()+QChar('#')+metaMethod.name());
    }
  }
}

int ConnectionPoint::objectId() const {
  return objectNode()->objectId();
}

ObjectGraphNode* ConnectionPoint::objectNode() const {
  return dynamic_cast<ObjectGraphNode*>(parentItem());
}

QObject* ConnectionPoint::object() const {
  return objectNode()->object();
}

QVariant ConnectionPoint::itemChange(GraphicsItemChange change, const QVariant &value)
{
  if (change == ItemScenePositionHasChanged) {
    QPointF position = value.toPointF();
    emit scenePositionChanged(position);
  }
  return QGraphicsItem::itemChange(change, value);
}


void ConnectionPoint::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,false);
  event->accept();
}

void ConnectionPoint::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,true);
  event->accept();
}

void ConnectionPoint::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {

  if (QLineF(event->screenPos(),
             event->buttonDownScreenPos(Qt::LeftButton)).length()
             < QApplication::startDragDistance()) {
    event->ignore();
    return;
  }

  setCursor(Qt::ClosedHandCursor);
  startDrag(event);
  setCursor(Qt::OpenHandCursor);
  event->accept();
}

void ConnectionPoint::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  qDebug() << Q_FUNC_INFO;
  setCursor(Qt::OpenHandCursor);
  setRect(rect().marginsAdded(QMargins(1,1,1,1)));
  update();
  event->accept();
}

void ConnectionPoint::dragLeaveEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO;
  setBrush(QBrush(Qt::gray));
  update();
  event->accept();
}


void ConnectionPoint::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  qDebug() << Q_FUNC_INFO;
  setBrush(QBrush(Qt::gray));
  setRect(rect().marginsRemoved(QMargins(1,1,1,1)));
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,true);
  update();
  event->accept();
}

