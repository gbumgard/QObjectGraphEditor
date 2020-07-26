#ifndef SETPROPERTYCOMMAND_H
#define SETPROPERTYCOMMAND_H


#include <QUndoCommand>
#include <QUuid>
#include <QVariant>

class ObjectGraph;

class SetPropertyCommand : public QUndoCommand
{

public:

    SetPropertyCommand(ObjectGraph* graph,
                       const QUuid& objectId,
                       const QString& propertyName,
                       const QVariant& prevValue,
                       const QVariant& nextValue,
                       QUndoCommand* parent = nullptr);

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

    virtual bool mergeWith(const QUndoCommand* other) override;

private:

  ObjectGraph* _graph;
  QUuid _objectId;
  QString _propertyName;
  QVariant _prevValue;
  QVariant _nextValue;

};


#endif // SETPROPERTYCOMMAND_H
