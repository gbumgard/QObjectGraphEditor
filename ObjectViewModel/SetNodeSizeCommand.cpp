#include "SetNodeSizeCommand.h"

#include <QDebug>

#include "ObjectGraph.h"
#include "ObjectGraphNode.h"


/**
 * @brief SetNodeSizeCommand
 * @param graph
 * @param objectId
 * @param size
 * @param parent
 */
SetNodeSizeCommand::SetNodeSizeCommand(ObjectGraph* graph,
                   const QUuid& objectUuid,
                   const QSizeF& newSize,
                   QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _objectUuid(objectUuid)
  , _oldSize(newSize)
  , _newSize(newSize)
{
}

int SetNodeSizeCommand::id() const {
  return ObjectGraph::SetNodeSizeCommandId;
}

void SetNodeSizeCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  ObjectGraphNode* node = _graph->node(_objectUuid);
  if (node) {
    _oldSize = node->size();
    node->resize(_newSize);
  }
  else {
    setObsolete(true);
  }
}

void SetNodeSizeCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  ObjectGraphNode* node = _graph->node(_objectUuid);
  if (node) {
    node->resize(_oldSize);
  }
  else {
    setObsolete(true);
  }
}

bool SetNodeSizeCommand::mergeWith(const QUndoCommand* other) {

  qDebug() << "\n" << Q_FUNC_INFO;

  if (other->id() != id())
    return false;

  const SetNodeSizeCommand* cmd = static_cast<const SetNodeSizeCommand*>(other);

  if (cmd && cmd->_objectUuid != _objectUuid)
    return false;

  _newSize = cmd->_newSize;

  return true;
}
