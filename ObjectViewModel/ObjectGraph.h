#ifndef NODEGRAPH_H
#define NODEGRAPH_H

#include "objectview_global.h"

#include <QGraphicsScene>
#include <QMap>
#include <QUndoStack>

#include "ObjectModel.h"

class ObjectGraphNode;
class ObjectGraphEdge;

/**
 * @brief The ObjectGraph class
 * The ::ObjectGraph class acts as a view model for an underlying ::ObjectModel instance.
 * Objects in the object model are represented as graph nodes and connections between signals and
 * slots as graph edges.
 */
class OBJECTVIEWSHARED_EXPORT ObjectGraph
  : public QGraphicsScene
{

  friend class ObjectGraphNode;
  friend class AddNodeCommand;
  friend class RemoveNodeCommand;
  friend class AddEdgeCommand;
  friend class RemoveEdgeCommand;
  friend class MoveNodesCommand;
  friend class SetNodeSizeCommand;
  friend class SetPropertyCommand;

  Q_OBJECT

public:

  ObjectGraph(QObject* parent = nullptr);

  virtual ~ObjectGraph() {}

  QString fileName() const { return _fileName; }
  void setFileName(const QString& fileName) { _fileName = fileName; }

  void clear();

  bool read(QDataStream& in);
  bool write(QDataStream& out) const;

  void setModel(ObjectModel* model);
  ObjectModel* model();

  void setCommandStack(QUndoStack* commandStack) {
    _commandStack = commandStack;
  }

  void addNode(const QString& className, const QPointF& position);
  void removeNode(int objectid);

  void addEdge(int senderId, int signalIndex, int receiverId, int slotIndex);
  void removeEdge(int connectionId);

  void moveNodes(const QList<int>& nodes, const QPointF& undoOffset);
  void setNodeSize(int objectid, const QSizeF& size);
  void setNodeGeometry(int objectid, const QRectF& size);
  void setProperty(int objectid,
                   const QString& propertyName,
                   const QVariant& oldValue,
                   const QVariant& newValue);

  static qreal topZValue();

private slots:

  void onObjectAdded(int objectId);
  void onObjectRemoved(int objectId);

  void onConnectionAdded(int connectionId);
  void onConnectionRemoved(int connectionId);

protected:

  int doAddNode(int objectId, const QString& className);
  bool doRemoveNode(int objectid);
  ObjectGraphNode* node(int objectId) const;

  int doAddEdge(int connectionId, int senderId, int signalIndex, int receiverId, int slotIndex);
  bool doRemoveEdge(int connectionId);
  ObjectGraphEdge* edge(int connectionId) const;

  static double _topZValue;

  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  void dragEnterEvent(QGraphicsSceneDragDropEvent *event) override;
  void dragLeaveEvent(QGraphicsSceneDragDropEvent *event) override;
  void dropEvent(QGraphicsSceneDragDropEvent *event) override;
  void dragMoveEvent(QGraphicsSceneDragDropEvent *event) override;
  void keyReleaseEvent(QKeyEvent * keyEvent) override;

private:

  QString _fileName;

  ObjectModel* _model;

  QMap<int,ObjectGraphNode*> _nodes;
  QMap<int,ObjectGraphEdge*> _edges;

  QUndoStack* _commandStack;

};

#endif // NODEGRAPH_H
