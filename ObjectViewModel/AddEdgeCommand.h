#ifndef ADDEDGECOMMAND_H
#define ADDEDGECOMMAND_H


#include <QUndoCommand>
#include <QUuid>

class ObjectGraph;

class AddEdgeCommand : public QUndoCommand
{

  public:

    explicit AddEdgeCommand(ObjectGraph* graph,
                            const QUuid& senderId,
                            const QString& signalSignature,
                            const QUuid& receiverId,
                            const QString& slotSignature,
                            QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

  private:

    ObjectGraph* _graph;
    QUuid _connectionUuid;
    QUuid _senderUuid;
    QString _signalSignature;
    QUuid _receiverUuid;
    QString _slotSignature;

};

#endif // ADDEDGECOMMAND_H
