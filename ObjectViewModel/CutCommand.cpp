#include "CutCommand.h"

#include "CutCommand.h"
#include "ObjectGraph.h"
#include "ObjectGraphNode.h"
#include "ObjectGraphEdge.h"


CutCommand::CutCommand(ObjectGraph* graph,
                       QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _nodeUuids()
  , _edgeUuids()
{
  QList<QGraphicsItem*> items = _graph->selectedItems();
  for (auto item : items) {
    ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
    if (node) {
      _nodeUuids.insert(node->uuid());
    }
    else {
      ObjectGraphEdge* edge = dynamic_cast<ObjectGraphEdge*>(item);
      if (edge) {
        _edgeUuids.insert(edge->uuid());
      }
    }
  }
}

int CutCommand::id() const {
  return ObjectGraph::CutCommandId;
}

void CutCommand::redo() {

  qDebug() << Q_FUNC_INFO;

  // Save the connection descriptions prior to deleting them

  _connections = _graph->connections(_edgeUuids);

  _graph->removeEdges(_edgeUuids);

  // Serialize all of the nodes (objects) and edges (connections) connecting them.
  // The some of the edges may have been deleted in the first step. Edges attached
  // to nodes being deleted are automatically deleted, so we need to save the nodes
  // and all of the connections attached to them.

  QDataStream out(_serializedGraph);

  _graph->serialize(out,_nodeUuids,true);

  _graph->removeNodes(_nodeUuids);
}

void CutCommand::undo() {

  qDebug() << Q_FUNC_INFO;

  QPointF centroid;

  // First reconstitute the saved nodes and any automatically deleted edges back into the graph.
  QDataStream in(&_serializedGraph,QIODevice::WriteOnly);

  _graph->deserialize(in,false,false,ObjectGraph::OriginalPlacement,centroid,_nodeUuids);

  for (auto connection : _connections) {
    _graph->addEdge(connection.connectionUuid(),
                    connection.senderUuid(),
                    connection.signalSignature(),
                    connection.receiverUuid(),
                    connection.slotSignature());
  }
}
