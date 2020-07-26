#ifndef SIZEGRIPPERWIDGET_H
#define SIZEGRIPPERWIDGET_H

#include <QGraphicsWidget>
#include <RoundedPolygonItem.h>
#include <QPen>
#include <QBrush>

class SizeGripperWidget : public QGraphicsWidget
{
  Q_OBJECT

public:

  SizeGripperWidget(const QSizeF& size, QGraphicsItem* parent);

  void setBrush(const QBrush& brush);
  QBrush brush() const { return _item->brush(); }

  void setPen(const QPen& pen);
  QPen pen() const { return _item->pen(); }

  QSizeF sizeHint(Qt::SizeHint which, const QSizeF &constraint) const override;

  QPainterPath shape() const override;

  QRectF boundingRect() const override;

protected:

  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

  void dragEnterEvent(QGraphicsSceneDragDropEvent *event) override;
  void dragLeaveEvent(QGraphicsSceneDragDropEvent *event) override;
  void dragMoveEvent(QGraphicsSceneDragDropEvent *event) override;
  void dropEvent(QGraphicsSceneDragDropEvent *event) override;

private:

  RoundedPolygonItem* _item;

};

#endif // SIZEGRIPPERWIDGET_H
