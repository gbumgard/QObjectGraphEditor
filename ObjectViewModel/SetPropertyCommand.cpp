#include "SetPropertyCommand.h"

#include <QDebug>

#include "ObjectGraph.h"

SetPropertyCommand::SetPropertyCommand(ObjectGraph* graph,
                   const QUuid& objectId,
                   const QString& propertyName,
                   const QVariant& prevValue,
                   const QVariant& nextValue,
                   QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _objectId(objectId)
  , _propertyName(propertyName)
  , _prevValue(prevValue)
  , _nextValue(nextValue)
{
  qDebug() << "\n" << Q_FUNC_INFO;
}

int SetPropertyCommand::id() const {
  return ObjectGraph::SetPropertyCommandId;
}

void SetPropertyCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  QObject* obj = _graph->model()->object(_objectId);
  if (obj) {
    obj->setProperty(_propertyName.toUtf8().constData(),_nextValue);
  }
  else {
    setObsolete(true);
  }
}

void SetPropertyCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  QObject* obj = _graph->model()->object(_objectId);
  if (obj) {
    obj->setProperty(_propertyName.toUtf8().constData(),_prevValue);
  }
  else {
    setObsolete(true);
  }
}

bool SetPropertyCommand::mergeWith(const QUndoCommand* other) {

  qDebug() << "\n" << Q_FUNC_INFO;

  if (other->id() != id())
    return false;

  const SetPropertyCommand* cmd = static_cast<const SetPropertyCommand*>(other);

  if (cmd->_objectId != _objectId)
    return false;

  _nextValue = cmd->_nextValue;

  return true;
}
