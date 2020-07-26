
#include "MoveNodesCommand.h"

#include "ObjectGraph.h"
#include "ObjectGraphNode.h"

MoveNodesCommand::MoveNodesCommand(ObjectGraph* graph,
                                   const QSet<QUuid>& nodeUuids,
                                   const QPointF& undoOffset,
                                   QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _nodeUuids(nodeUuids)
  , _undoOffset(undoOffset)
  , _redoOffset(0,0)
{
  qDebug() << "\n" << Q_FUNC_INFO << nodeUuids << undoOffset;
}

int MoveNodesCommand::id() const {
  return ObjectGraph::MoveNodesCommandId;
};

void MoveNodesCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  for (auto objectId : _nodeUuids) {
    ObjectGraphNode* node = _graph->node(objectId);
    if (node) {
      node->moveBy(_redoOffset.x(),_redoOffset.y());
    }
  }
}

void MoveNodesCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  for (auto objectId : _nodeUuids) {
    ObjectGraphNode* node = _graph->node(objectId);
    if (node) {
      node->moveBy(_undoOffset.x(),_undoOffset.y());
    }
  }
  _redoOffset = QPointF(-_undoOffset.x(),-_undoOffset.y());
}

bool MoveNodesCommand::mergeWith(const QUndoCommand* other) {

  qDebug() << "\n" << Q_FUNC_INFO;

  if (other->id() != id()) return false;

  const MoveNodesCommand* cmd = static_cast<const MoveNodesCommand*>(other);

  if (cmd && cmd->_nodeUuids != _nodeUuids) return false;

  _undoOffset = cmd->_undoOffset + _undoOffset;

  return true;
}
