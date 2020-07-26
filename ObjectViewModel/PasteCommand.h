#ifndef PASTECOMMAND_H
#define PASTECOMMAND_H

#include <QUndoCommand>
#include <QSet>
#include <QPointF>

#include "ObjectGraph.h"

class PasteCommand : public QUndoCommand
{

  public:

    explicit PasteCommand(ObjectGraph* graph,
                          ObjectGraph::Placement placement,
                          const QPointF& position,
                          QUndoCommand* parent = nullptr);


    virtual ~PasteCommand() {}

    virtual int id() const override;

    virtual void redo() override;

    virtual void undo() override;

  private:

    ObjectGraph* _graph;
    ObjectGraph::Placement _placement;
    QPointF _position;
    QPointF _initialOffset;
    QSet<QUuid> _nodeUuids;
};

#endif // PASTECOMMAND_H
