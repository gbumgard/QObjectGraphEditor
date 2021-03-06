#ifndef CONNECTIONPATH_H
#define CONNECTIONPATH_H

#include <QObject>
#include <QGraphicsPathItem>

class ObjectGraphEdge : public QObject, public QGraphicsPathItem
{

  Q_OBJECT

public:

  ObjectGraphEdge(int connectionId = -1);

  virtual ~ObjectGraphEdge();

  void setEndpoints(const QPointF& signalPosition, const QPointF& slotPosition);

  const QPointF& signalPosition() const { return _signalPosition; }
  const QPointF& slotPosition() const { return _slotPosition; }

  int connectionId() const { return _connectionId; }

public slots:

  void setSignalPosition(const QPointF& position);
  void setSlotPosition(const QPointF& position);

protected:

  void updatePath();

  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;

private:

  QPointF _signalPosition;
  QPointF _slotPosition;
  int _connectionId;

};

#endif // CONNECTIONPATH_H
