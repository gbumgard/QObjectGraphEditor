#include "PasteCommand.h"

#include <QApplication>
#include <QMimeData>
#include <QClipboard>
#include <QDebug>

#include "ObjectGraph.h"

PasteCommand::PasteCommand(ObjectGraph* graph,
                           ObjectGraph::Placement placement,
                           const QPointF& position,
                           QUndoCommand* parent)
  : QUndoCommand(parent)
  , _graph(graph)
  , _placement(placement)
  , _position(position)
  , _nodeUuids()
{
  qDebug() << "\n" << Q_FUNC_INFO;
}

int PasteCommand::id() const {
  return ObjectGraph::PasteCommandId;
}

void PasteCommand::redo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  const QMimeData* mimeData = QApplication::clipboard()->mimeData();

  QStringList formats = mimeData->formats();

  for (auto format : formats) {

    if (format == ObjectGraph::SERIALIZED_GRAPH_MIME_TYPE) {

      QSet<QUuid> nodeUuids;
      QByteArray buffer = mimeData->data(ObjectGraph::SERIALIZED_GRAPH_MIME_TYPE);
      QDataStream in(buffer);
      _graph->deserialize(in,true,_nodeUuids.empty(),_placement,_position,_nodeUuids);
    }
  }
}

void PasteCommand::undo() {

  qDebug() << "\n" << Q_FUNC_INFO;

  _graph->removeNodes(_nodeUuids);

}
