#include "SlotConnectionPoint.h"

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

SlotConnectionPoint::SlotConnectionPoint(const QMetaMethod& metaMethod,
                                         QGraphicsItem* parent)
  : ConnectionPoint(metaMethod,parent)
{
}

void SlotConnectionPoint::startDrag(QGraphicsSceneMouseEvent *event) {
  qDebug() << Q_FUNC_INFO;
  ObjectGraphEdge* edge = new ObjectGraphEdge();
  edge->setSignalPosition(scenePos());
  edge->setSlotPosition(scenePos());
  scene()->addItem(edge);
  QDrag *drag = new QDrag(event->widget());
  SlotMimeData* mimeData = new SlotMimeData(objectUuid(),_metaMethod.methodSignature(),edge);
  drag->setMimeData(mimeData);
  drag->exec();
  scene()->removeItem(edge);
  edge->deleteLater();
  event->accept();
}

void SlotConnectionPoint::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->mimeData()->formats();
  if (event->mimeData()->hasFormat(SignalMimeData::MIME_TYPE)) {
    const SignalMimeData* mimeData = dynamic_cast<const SignalMimeData*>(event->mimeData());
    if (mimeData) {
      qDebug() << Q_FUNC_INFO << mimeData->objectUuid() << mimeData->methodSignature() << objectUuid() << _metaMethod.methodSignature();
      if (objectNode()->model()->canConnect(mimeData->objectUuid(),
                                            mimeData->methodSignature(),
                                            objectUuid(),
                                            _metaMethod.methodSignature())) {
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

void SlotConnectionPoint::dropEvent(QGraphicsSceneDragDropEvent *event) {
  qDebug() << Q_FUNC_INFO << event->mimeData()->formats();
  setBrush(QBrush(Qt::gray));
  if (event->mimeData()->hasFormat(SignalMimeData::MIME_TYPE)) {
    const SignalMimeData* mimeData = dynamic_cast<const SignalMimeData*>(event->mimeData());
    if (mimeData) {
      if (objectNode()->model()->canConnect(mimeData->objectUuid(),
                                            mimeData->methodSignature(),
                                            objectUuid(),
                                            _metaMethod.methodSignature())) {
        objectNode()->graph()->addEdgeAction(mimeData->objectUuid(),
                                             mimeData->methodSignature(),
                                             objectUuid(),
                                             _metaMethod.methodSignature());
        event->acceptProposedAction();
        return;
      }
    }
  }
  event->setDropAction(Qt::IgnoreAction);
}

