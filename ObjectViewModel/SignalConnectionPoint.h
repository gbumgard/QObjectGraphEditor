#ifndef SIGNALCONNECTIONPOINT_H
#define SIGNALCONNECTIONPOINT_H

#include "ConnectionPoint.h"

class SignalConnectionPoint : public ConnectionPoint
{
public:

  SignalConnectionPoint(const QMetaMethod& metaMethod,
                        QGraphicsItem* parent = nullptr);

protected:

  void startDrag(QGraphicsSceneMouseEvent *event) override;

  virtual void dragEnterEvent(QGraphicsSceneDragDropEvent *event) override;
  virtual void dropEvent(QGraphicsSceneDragDropEvent *event) override;

};
#endif // SIGNALCONNECTIONPOINT_H
