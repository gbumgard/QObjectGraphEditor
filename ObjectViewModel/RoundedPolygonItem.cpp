#include "RoundedPolygonItem.h"
#include <math.h>
#include <QDebug>

float RoundedPolygonItem::getDistance(QPointF pt1, QPointF pt2) const
{
    float fD = (pt1.x() - pt2.x())*(pt1.x() - pt2.x()) +
               (pt1.y() - pt2.y()) * (pt1.y() - pt2.y());
    return sqrt(fD);
}

QPointF RoundedPolygonItem::getLineStart(int i) const
{
    QPointF pt;
    QPointF pt1 = at(i);
    QPointF pt2 = at((i+1) % count());
    float fRat = _radius / getDistance(pt1, pt2);

    if (fRat > 0.5f) fRat = 0.5f;

    pt.setX((1.0f-fRat)*pt1.x() + fRat*pt2.x());
    pt.setY((1.0f-fRat)*pt1.y() + fRat*pt2.y());
    return pt;
}
QPointF RoundedPolygonItem::getLineEnd(int i) const
{
    QPointF pt;
    QPointF pt1 = at(i);
    QPointF pt2 = at((i+1) % count());
    float fRat = _radius / getDistance(pt1, pt2);
    if (fRat > 0.5f)
     fRat = 0.5f;
    pt.setX(fRat*pt1.x() + (1.0f - fRat)*pt2.x());
    pt.setY(fRat*pt1.y() + (1.0f - fRat)*pt2.y());
    return pt;
}

void RoundedPolygonItem::updatePath()
{
    QPainterPath path;

    if (count() < 3) {
      qWarning() << "Polygon should have at least 3 points!";
      setPath(QPainterPath());
    }

    QPointF pt1;
    QPointF pt2;
    for (int i = 0; i < count(); i++) {
     pt1 = getLineStart(i);

     if (i == 0)
       path.moveTo(pt1);
     else
       path.quadTo(at(i), pt1);

     pt2 = getLineEnd(i);
     path.lineTo(pt2);
    }

    // close the last corner
    pt1 = getLineStart(0);
    path.quadTo(at(0), pt1);

    setPath(path);
}
