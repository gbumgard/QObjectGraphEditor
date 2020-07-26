#ifndef ADDNODECOMMAND_H
#define ADDNODECOMMAND_H


#include <QUndoCommand>
#include <QUuid>
#include <QPointF>

class ObjectGraph;

class AddNodeCommand : public QUndoCommand
{

  public:

    explicit AddNodeCommand(ObjectGraph* graph,
                            const QString& className,
                            const QPointF& position,
                            QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

  private:

    ObjectGraph* _graph;
    QUuid _nodeUuid;
    QString _className;
    QPointF _position;

};


#endif // ADDNODECOMMAND_H
