#include "SignalConnectionPoint.h"

#include <QDrag>
#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

#include <QDebug>

#include "ObjectModel.h"
#include "ObjectGraph.h"
#include "ObjectGraphNode.h"
#include "ObjectGraphEdge.h"

#include "SlotMimeData.h"
#include "SignalMimeData.h"

SignalConnectionPoint::SignalConnectionPoint(const QMetaMethod& metaMethod,
                                             QGraphicsItem* parent)
  : ConnectionPoint(metaMethod,parent)
{
}

void SignalConnectionPoint::startDrag(QGraphicsSceneMouseEvent *event) {
  qDebug() << Q_FUNC_INFO;
  ObjectGraphEdge* edge = new ObjectGraphEdge();
  edge->setSignalPosition(scenePos());
  edge->setSlotPosition(scenePos());
  scene()->addItem(edge);
  QDrag *drag = new QDrag(event->widget());
  SignalMimeData* mimeData = new SignalMimeData(objectId(),methodIndex(),edge);
  drag->setMimeData(mimeData);
  drag->exec();
  scene()->removeItem(edge);
  edge->deleteLater();
  event->accept();
}

void SignalConnectionPoint::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->mimeData()->formats();
  if (event->mimeData()->hasFormat(SlotMimeData::MIME_TYPE)) {
    const SlotMimeData* mimeData = dynamic_cast<const SlotMimeData*>(event->mimeData());
    if (mimeData) {
      qDebug() << Q_FUNC_INFO << mimeData->objectId() << mimeData->methodIndex() << objectId() << methodIndex();
      if (objectNode()->model()->canConnect(objectId(),
                                            methodIndex(),
                                            mimeData->objectId(),
                                            mimeData->methodIndex())) {
        qDebug() << "CAN CONNECT";
        setBrush(QBrush(Qt::green));
      }
      else {
        qDebug() << "CANNOT CONNECT";
        setBrush(QBrush(Qt::red));
      }
      event->accept();
      return;
    }
  }
  event->ignore();
}

void SignalConnectionPoint::dropEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->mimeData()->formats();
  setBrush(QBrush(Qt::gray));
  if (event->mimeData()->hasFormat(SlotMimeData::MIME_TYPE)) {
    const SlotMimeData* mimeData = dynamic_cast<const SlotMimeData*>(event->mimeData());
    if (mimeData) {
      if (objectNode()->model()->canConnect(objectId(),
                                            methodIndex(),
                                            mimeData->objectId(),
                                            mimeData->methodIndex())) {
        objectNode()->graph()->addEdge(objectId(),
                                       methodIndex(),
                                       mimeData->objectId(),
                                       mimeData->methodIndex());
        event->acceptProposedAction();
        return;
      }
    }
  }
  event->setDropAction(Qt::IgnoreAction);
}

