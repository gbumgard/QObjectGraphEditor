#ifndef SLOTCONNECTIONPOINT_H
#define SLOTCONNECTIONPOINT_H

#include "ConnectionPoint.h"

class SlotConnectionPoint : public ConnectionPoint
{
public:

  SlotConnectionPoint(const QMetaMethod& metaMethod,
                      QGraphicsItem* parent = nullptr);

protected:

  void startDrag(QGraphicsSceneMouseEvent *event) override;

  virtual void dragEnterEvent(QGraphicsSceneDragDropEvent *event) override;
  virtual void dropEvent(QGraphicsSceneDragDropEvent *event) override;

};

#endif // SLOTCONNECTIONPOINT_H
