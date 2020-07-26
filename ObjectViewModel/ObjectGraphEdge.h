#ifndef CONNECTIONPATH_H
#define CONNECTIONPATH_H

#include <QObject>
#include <QGraphicsPathItem>
#include <QUuid>

class ObjectGraphEdge : public QObject, public QGraphicsPathItem
{

  Q_OBJECT

public:

  ObjectGraphEdge(const QUuid& connectionUuid = QUuid());

  virtual ~ObjectGraphEdge();

  void setEndpoints(const QPointF& signalPosition, const QPointF& slotPosition);

  const QPointF& signalPosition() const { return _signalPosition; }
  const QPointF& slotPosition() const { return _slotPosition; }

  QUuid uuid() const { return connectionUuid(); }
  QUuid connectionUuid() const { return _connectionUuid; }

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
  QUuid _connectionUuid;

};

#endif // CONNECTIONPATH_H
