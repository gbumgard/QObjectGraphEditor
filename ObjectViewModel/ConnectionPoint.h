#ifndef NODEPAD_H
#define NODEPAD_H

#include <QGraphicsEllipseItem>
#include <QMetaMethod>

class ObjectGraphNode;

class ConnectionPoint : public QObject, public QGraphicsEllipseItem
{

  Q_OBJECT

public:

  static constexpr qreal RADIUS = 6.0;

  ConnectionPoint(const QMetaMethod& metaMethod,
                  QGraphicsItem* parent = nullptr);


  ConnectionPoint(const ConnectionPoint&) = delete;
  ConnectionPoint& operator=(const ConnectionPoint&) = delete;

  virtual ~ConnectionPoint() {}

  QUuid objectUuid() const;
  ObjectGraphNode* objectNode() const;
  QObject* object() const;

  int methodIndex() const { return _metaMethod.methodIndex(); }
  QMetaMethod metaMethod() const { return _metaMethod; }

signals:

  void scenePositionChanged(const QPointF& sceneCoordinates);

protected:

  virtual void startDrag(QGraphicsSceneMouseEvent* event) = 0;

  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

  virtual void dragLeaveEvent(QGraphicsSceneDragDropEvent *event) override;

protected:

  QMetaMethod _metaMethod;

};

#endif // NODEPAD_H
