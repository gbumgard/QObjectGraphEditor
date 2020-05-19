#include "ObjectGraphEdge.h"
#include "ObjectGraph.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneHoverEvent>
#include <QColor>

#include <QDebug>

ObjectGraphEdge::ObjectGraphEdge(int connectionId)
  : QObject()
  , QGraphicsPathItem()
  , _connectionId(connectionId)
{
  setZValue(ObjectGraph::topZValue());
  setFlag(QGraphicsItem::ItemIsSelectable,true);
  setAcceptHoverEvents(true);
}

ObjectGraphEdge::~ObjectGraphEdge() {
  if (scene()) scene()->removeItem(this);
}

void ObjectGraphEdge::paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {

  painter->setRenderHint(QPainter::Antialiasing);
  QPen edgePen;
  edgePen.setWidth(4);
  edgePen.setColor(isSelected() ? QColor("orange") : QColor(Qt::gray));
  edgePen.setCapStyle(Qt::RoundCap);
  setPen(edgePen);
  const_cast<QStyleOptionGraphicsItem *>(option)->state.setFlag(QStyle::State_Selected,false);
  QGraphicsPathItem::paint(painter,option,widget);
}

void ObjectGraphEdge::setEndpoints(const QPointF& signalPadPosition, const QPointF& slotPadPosition) {
  _signalPosition = signalPadPosition;
  _slotPosition = slotPadPosition;
  updatePath();
}

void ObjectGraphEdge::setSignalPosition(const QPointF& signalPadPosition) {
  _signalPosition = signalPadPosition;
  updatePath();
}

void ObjectGraphEdge::setSlotPosition(const QPointF& slotPadPosition) {
  _slotPosition = slotPadPosition;
   updatePath();
}

void ObjectGraphEdge::updatePath() {

  double xDistance = _slotPosition.x() - _signalPosition.x();
  double defaultOffset = 180;
  double minimum = qMin(defaultOffset, qAbs(xDistance));
  double verticalOffset = 0;
  double ratio1 = 0.5;

  if (xDistance <= 0) {
    verticalOffset = -minimum;
    ratio1 = 1.0;
  }

  QPointF c1(_signalPosition.x() + minimum * ratio1,
             _signalPosition.y() + verticalOffset);

  QPointF c2(_slotPosition.x() - minimum * ratio1,
             _slotPosition.y() + verticalOffset);

  QPainterPath cubic(_signalPosition);

  cubic.cubicTo(c1, c2, _slotPosition);

  setPath(cubic);

  update();
}

QVariant ObjectGraphEdge::itemChange(GraphicsItemChange change, const QVariant &value) {
  if (change == QGraphicsItem::ItemSelectedChange) {
    if (value.toBool()) {
      setZValue(ObjectGraph::topZValue());
      update();
    }
  }
  return QGraphicsPathItem::itemChange(change,value);
}

void ObjectGraphEdge::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  setCursor(Qt::ArrowCursor);
  event->accept();
}

void ObjectGraphEdge::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  event->accept();
}
