#include "DeleteCommand.h"
#include "ObjectGraph.h"
#include "ObjectGraphNode.h"
#include "ObjectGraphEdge.h"

/**
 * @brief RemoveEdgeCommand
 * @param graph
 * @param connectionId
 * @param parent
 */
DeleteCommand::DeleteCommand(ObjectGraph* graph,
                             QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _nodeUuids()
  , _edgeUuids()
  , _serializedGraph()
{

  qDebug() << "\n" << Q_FUNC_INFO;

  QList<QGraphicsItem*> items = _graph->selectedItems();
  for (auto item : items) {
    ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
    if (node) {
      qDebug() << "flagging node for deletion" << node->uuid();
      _nodeUuids.insert(node->uuid());
    }
    else {
      ObjectGraphEdge* edge = dynamic_cast<ObjectGraphEdge*>(item);
      if (edge) {
        qDebug() << "flagging edge for deletion" << edge->uuid();
        _edgeUuids.insert(edge->uuid());
      }
    }
  }

  qDebug() << "\nDELETE command targets";
  qDebug() << "nodes:" <<_nodeUuids;
  qDebug() << "edges:" << _edgeUuids;
  qDebug() << "\n";

}


int DeleteCommand::id() const {
  return ObjectGraph::DeleteCommandId;
}

void DeleteCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  if (!_edgeUuids.isEmpty()) {
    // Save the connection descriptions prior to deleting them

    _connections = _graph->connections(_edgeUuids);

    _graph->removeEdges(_edgeUuids);
  }

  // Serialize all of the nodes (objects) and edges (connections) connecting them.
  // The some of the edges may have been deleted in the first step. Edges attached
  // to nodes being deleted are automatically deleted, so we need to save the nodes
  // and all of the connections attached to them.

  if (!_nodeUuids.isEmpty()) {

    QDataStream out(&_serializedGraph,QIODevice::WriteOnly);

    _graph->serialize(out,_nodeUuids,true);

    _graph->removeNodes(_nodeUuids);

  }
}

/**
 * @brief undo
 */
void DeleteCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  if (!_nodeUuids.isEmpty()) {

    QPointF centroid;

    // First reconstitute the saved nodes and any automatically deleted edges back into the graph.

    QDataStream in(_serializedGraph);

    _graph->deserialize(in,false,false,ObjectGraph::OriginalPlacement,centroid,_nodeUuids);

  }

  if (!_connections.isEmpty()) {
    for (auto connection : _connections) {
      _graph->addEdge(connection.connectionUuid(),
                      connection.senderUuid(),
                      connection.signalSignature(),
                      connection.receiverUuid(),
                      connection.slotSignature());
    }
  }
}
