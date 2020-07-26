#ifndef ROUNDEDPOLYGONITEM_H
#define ROUNDEDPOLYGONITEM_H

#include <QGraphicsPathItem>
#include <QPointF>
#include <QPainterPath>

class RoundedPolygonItem : public QGraphicsPathItem
{
public:

  RoundedPolygonItem(QGraphicsItem* parent = nullptr)
    : QGraphicsPathItem(parent)
    , _polygon()
    , _radius(0.0)
  {
  }

  void setPolygon(const QPolygonF& polygon, qreal radius = 0.0) {
    _polygon = polygon;
    _radius = radius;
    updatePath();
  }

  void setRadius(qreal radius)
  {
    _radius = radius;
  }

private:

  void updatePath();

  QPointF getLineStart(int i) const;

  QPointF getLineEnd(int i) const;

  float getDistance(QPointF pt1, QPointF pt2) const;

  QPointF at(int index) const {
    return _polygon.at(index);
  }

  int count() const {
    return _polygon.count();
  }

  QPolygonF _polygon;
  qreal _radius;

};

#endif // ROUNDEDPOLYGONITEM_H
