#include "SizeGripperWidget.h"

#include <QApplication>
#include <QGraphicsSceneHoverEvent>
#include <QDrag>
#include <QGraphicsScene>
#include <QWidget>

#include "GraphicsItemMimeData.h"

#include <QDebug>

SizeGripperWidget::SizeGripperWidget(const QSizeF& size, QGraphicsItem* parent)
  : QGraphicsWidget(parent)
  , _item(new RoundedPolygonItem(this))
{
  QPolygonF poly;
  poly << QPointF(size.width(),0.0) << QPointF(size.width(),size.height()) << QPointF(0,size.height());
  _item->setPolygon(poly,4.0);
  setAcceptDrops(true);
  setFlags(QGraphicsItem::ItemSendsScenePositionChanges | QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable);
  setAcceptHoverEvents(true);
  setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
}

void SizeGripperWidget::setBrush(const QBrush& brush) {
  _item->setBrush(brush);
}

void SizeGripperWidget::setPen(const QPen& pen) {
  prepareGeometryChange();
  _item->setPen(pen);
  updateGeometry();
}

QSizeF SizeGripperWidget::sizeHint(Qt::SizeHint which, const QSizeF &constraint) const {
  QSizeF hint;
  switch (which) {
    case Qt::MinimumSize:
      hint = QSizeF(0,0);
      break;
    case Qt::PreferredSize:
    {
      hint = boundingRect().size();
      break;
    }
    case Qt::MaximumSize:
    default:
      hint = QSizeF(16777215,16777215);
      break;
  }
  if (constraint.width() != -1) hint.setWidth(constraint.width());
  if (constraint.height() != -1) hint.setHeight(constraint.height());
  qDebug() << Q_FUNC_INFO << which << constraint << hint;
  return hint;
}

QRectF SizeGripperWidget::boundingRect() const {
  qreal left, top, right, bottom;
  getContentsMargins(&left,&top,&right,&bottom);
  QRectF rect(QPointF(0,0),QSizeF(_item->boundingRect().width()+left+right,_item->boundingRect().height()+top+bottom));
  qDebug() << _item->boundingRect() << left << top << right << bottom << rect;
  return rect;
}

QPainterPath SizeGripperWidget::shape() const {
  return _item->shape();
}

void SizeGripperWidget::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos() << event->screenPos();

  if (QLineF(event->screenPos(),
             event->buttonDownScreenPos(Qt::LeftButton)).length()
             < QApplication::startDragDistance()) {
    event->ignore();
    return;
  }

  setCursor(Qt::SizeAllCursor);

  QDrag *drag = new QDrag(event->widget());
  QMimeData* mimeData = new GraphicsItemMimeData(this->parentItem(),
                                                 QStringLiteral("application/x-graphicsitem-size"));
  qDebug() << Q_FUNC_INFO << "Start drag resize operation" << mimeData->formats();
  drag->setMimeData(mimeData);
  drag->exec();
  setCursor(Qt::OpenHandCursor);
  event->accept();
}

void SizeGripperWidget::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,false);
  parentItem()->scene()->clearSelection();
  parentItem()->setSelected(true);
  event->accept();
}

void SizeGripperWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,true);
  event->accept();
}

void SizeGripperWidget::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  event->accept();
  setCursor(Qt::SizeFDiagCursor);
  QGraphicsWidget::hoverEnterEvent(event);
}

void SizeGripperWidget::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  event->accept();
  parentItem()->setFlag(QGraphicsItem::ItemIsMovable,true);
  QGraphicsWidget::hoverLeaveEvent(event);
}

void SizeGripperWidget::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  QGraphicsWidget::dragEnterEvent(event);
}

void SizeGripperWidget::dragLeaveEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  QGraphicsWidget::dragLeaveEvent(event);
}

void SizeGripperWidget::dragMoveEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  QGraphicsWidget::dragMoveEvent(event);
}

void SizeGripperWidget::dropEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->pos() << event->scenePos();
  QGraphicsWidget::dropEvent(event);
}
