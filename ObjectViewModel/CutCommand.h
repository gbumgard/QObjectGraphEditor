#ifndef CUTCOMMAND_H
#define CUTCOMMAND_H

#include <QUndoCommand>
#include <QUuid>
#include <QSet>

#include "Connection.h"

class ObjectGraph;

class CutCommand : public QUndoCommand
{

  public:

    explicit CutCommand(ObjectGraph* graph,
                        QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

  private:

    ObjectGraph* _graph;
    QByteArray _serializedGraph;
    QSet<QUuid> _nodeUuids;
    QSet<QUuid> _edgeUuids;
    QSet<Connection> _connections;

};


#endif // CUTCOMMAND_H
