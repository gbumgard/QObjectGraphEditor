#ifndef MOVECOMMAND_H
#define MOVECOMMAND_H


#include <QUndoCommand>
#include <QUuid>
#include <QSet>
#include <QPointF>

class ObjectGraph;

class MoveNodesCommand : public QUndoCommand
{

  public:

    MoveNodesCommand(ObjectGraph* graph,
                     const QSet<QUuid>& nodeUuids,
                     const QPointF& undoOffset,
                     QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

    virtual bool mergeWith(const QUndoCommand* other) override;

  protected:

    ObjectGraph* _graph;
    QSet<QUuid> _nodeUuids;
    QPointF _undoOffset;
    QPointF _redoOffset;

};

#endif // MOVECOMMAND_H
