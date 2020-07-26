#include "AddNodeCommand.h"

#include <QString>
#include <QPointF>

#include <typeinfo>

#include "ObjectGraph.h"
#include "ObjectGraphNode.h"

AddNodeCommand::AddNodeCommand(ObjectGraph* graph,
                               const QString& className,
                               const QPointF& position,
                               QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _nodeUuid(QUuid::createUuid())
  , _className(className)
  , _position(position)
{
  qDebug() << "\n" << Q_FUNC_INFO << className << position;
}

int AddNodeCommand::id() const {
  return ObjectGraph::AddNodeCommandId;
}

void AddNodeCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  _graph->addNode(_nodeUuid,_className);
  ObjectGraphNode* node = _graph->node(_nodeUuid);
  if (node) node->setPos(_position);
}

void AddNodeCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  QSet<QUuid> nodeUuids;
  nodeUuids += _nodeUuid;
  _graph->removeNodes(nodeUuids);
}

