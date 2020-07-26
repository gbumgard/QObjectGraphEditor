#ifndef SETNODESIZECOMMAND_H
#define SETNODESIZECOMMAND_H

#include <QUndoCommand>
#include <QUuid>
#include <QSizeF>

class ObjectGraph;

class SetNodeSizeCommand : public QUndoCommand
{

  public:

    SetNodeSizeCommand(ObjectGraph* graph,
                       const QUuid& objectId,
                       const QSizeF& newSize,
                       QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

    virtual bool mergeWith(const QUndoCommand* other) override;

  private:

    ObjectGraph* _graph;
    QUuid _objectUuid;
    QSizeF _oldSize;
    QSizeF _newSize;

};


#endif // SETNODESIZECOMMAND_H
