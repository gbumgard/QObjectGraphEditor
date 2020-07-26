#include "AddEdgeCommand.h"
#include <QDebug>

#include "ObjectGraph.h"


AddEdgeCommand::AddEdgeCommand(ObjectGraph* graph,
               const QUuid& senderUuid,
               const QString& signalSignature,
               const QUuid& receiverUuid,
               const QString& slotSignature,
               QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _connectionUuid(QUuid::createUuid())
  , _senderUuid(senderUuid)
  , _signalSignature(signalSignature)
  , _receiverUuid(receiverUuid)
  , _slotSignature(slotSignature)
{
  qDebug() << "\n" << Q_FUNC_INFO << "sender:" << senderUuid << "signal-sig:" << signalSignature << "receiver:" << receiverUuid << "slot-sig:" << slotSignature;
}

int AddEdgeCommand::id() const {
  return ObjectGraph::AddEdgeCommandId;
}

void AddEdgeCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  _graph->addEdge(_connectionUuid,_senderUuid,_signalSignature,_receiverUuid,_slotSignature);
}

void AddEdgeCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  QSet<QUuid> connectionUuids;
  connectionUuids.insert(_connectionUuid);
  _graph->removeEdges(connectionUuids);
  if (_connectionUuid.isNull()) setObsolete(true);
}
