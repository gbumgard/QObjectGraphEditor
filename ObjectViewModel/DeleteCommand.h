#ifndef REMOVEEDGESCOMMAND_H
#define REMOVEEDGESCOMMAND_H


#include <QUndoCommand>
#include <QUuid>
#include <QSet>

#include "Connection.h"

class QGraphicsItem;
class ObjectGraph;

class DeleteCommand : public QUndoCommand
{

  public:

    explicit DeleteCommand(ObjectGraph* graph,
                           QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

  private:

    ObjectGraph* _graph;
    QSet<QUuid> _nodeUuids;
    QSet<QUuid> _edgeUuids;
    QSet<Connection> _connections;
    QByteArray _serializedGraph;

};

#endif // REMOVEEDGESCOMMAND_H
