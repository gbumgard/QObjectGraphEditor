#include "SizeGripper.h"

#include <QApplication>
#include <QDrag>
#include <QMimeData>
#include <QWidget>
#include <QGraphicsScene>
#include "GraphicsItemMimeData.h"
#include <QDebug>


SizeGripper::SizeGripper(const QSizeF& size, QGraphicsItem* parent)
  : RoundedPolygonItem(parent)
{
  QPolygonF poly;
  poly << QPointF(size.width(),0.0) << QPointF(size.width(),size.height()) << QPointF(0,size.height());
  setPolygon(poly,4.0);
  setAcceptDrops(true);
  setFlag(QGraphicsItem::ItemSendsScenePositionChanges);
  setAcceptHoverEvents(true);
}

void SizeGripper::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos() << event->screenPos();

  if (QLineF(event->screenPos(),
             event->buttonDownScreenPos(Qt::LeftButton)).length()
             < QApplication::startDragDistance()) {
    event->ignore();
    return;
  }

  setCursor(Qt::DragLinkCursor);

  QDrag *drag = new QDrag(event->widget());
  QMimeData* mimeData = new GraphicsItemMimeData(this->parentItem(),
                                                 QStringLiteral("application/x-graphicsitem-size"));
  qDebug() << Q_FUNC_INFO << "Start drag resize operation" << mimeData->formats();
  drag->setMimeData(mimeData);
  drag->exec();
  setCursor(Qt::OpenHandCursor);
  event->accept();
}

void SizeGripper::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,false);
  parentItem()->scene()->clearSelection();
  parentItem()->setSelected(true);
  event->accept();
}

void SizeGripper::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,true);
  event->accept();
}

void SizeGripper::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  event->accept();
  setCursor(Qt::SizeFDiagCursor);
  RoundedPolygonItem::hoverEnterEvent(event);
}

void SizeGripper::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  event->accept();
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,true);
  RoundedPolygonItem::hoverLeaveEvent(event);
}

void SizeGripper::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  RoundedPolygonItem::dragEnterEvent(event);
}

void SizeGripper::dragLeaveEvent(QGraphicsSceneDragDropEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  RoundedPolygonItem::dragLeaveEvent(event);
}

void SizeGripper::dragMoveEvent(QGraphicsSceneDragDropEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  RoundedPolygonItem::dragMoveEvent(event);
}

void SizeGripper::dropEvent(QGraphicsSceneDragDropEvent *event) {
  //qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  RoundedPolygonItem::dropEvent(event);
}
