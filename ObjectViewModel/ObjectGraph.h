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
    friend class AddEdgeCommand;
    friend class MoveNodesCommand;
    friend class SetNodePositionCommand;
    friend class SetNodeSizeCommand;
    friend class SetPropertyCommand;
    friend class CutCommand;
    friend class CopyCommand;
    friend class PasteCommand;
    friend class DeleteCommand;

    Q_OBJECT


    enum CommandIds {
      AddNodeCommandId,
      AddEdgeCommandId,
      MoveNodesCommandId,
      SetNodePositionCommandId,
      SetNodeSizeCommandId,
      SetPropertyCommandId,
      CutCommandId,
      PasteCommandId,
      DeleteCommandId
    };

  public:

    enum Placement {
      OriginalPlacement,
      AbsolutePlacement,
      OffsetPlacement
    };

    static constexpr const char* SERIALIZED_GRAPH_MIME_TYPE = "application/x-qgr-serialized-graph";

    ObjectGraph(QObject* parent = nullptr);

    virtual ~ObjectGraph();

    QString fileName() const { return _fileName; }

    void setFileName(const QString& fileName) { _fileName = fileName; }

    void clear();

    bool deserialize(QDataStream& in,
                     bool selectNodes,
                     bool makeUnique,
                     Placement placement,
                     QPointF& position,
                     QSet<QUuid>& nodeUuids);

    bool serialize(QDataStream& out,
                   QSet<QUuid>& nodeUuids,
                   bool allConnections) const;

    void setModel(ObjectModel* model);
    ObjectModel* model();

    void setCommandStack(QUndoStack* commandStack) {
      _commandStack = commandStack;
    }

    void addNodeAction(const QString& className, const QPointF& position);

    void addEdgeAction(const QUuid& senderUuid,
                       const QString& signalSignature,
                       const QUuid& receiverUuid,
                       const QString& slotSignature);

    static qreal topNodeZValue();

    static qreal topEdgeZValue();

  public slots:

    void onUndoAction();

    void onRedoAction();

    void onCutAction();

    void onCopyAction();

    void onPasteAction();

    void onPasteAction(const QPointF& scenePosition);

    void onDeleteAction();

    void onSelectAllAction();

    void onContextHelpAction();

  private slots:

    void onObjectAdded(const QUuid &objectUuid);

    void onObjectRemoved(const QUuid& objectUuid);

    void onConnectionAdded(const QUuid &connectionUuid);

    void onConnectionRemoved(const QUuid& connectionUuid);

    void onObjectStatusChanged(const QUuid &objectUuid);

  protected:

    void addNode(const QUuid &objectId, const QString& className);

    void removeNodes(QSet<QUuid>& nodeUuids);

    void addEdge(const QUuid& connectionUuid,
                 const QUuid& senderUuid,
                 const QString& signalSignature,
                 const QUuid& receiverUuid,
                 const QString& slotSignature);

    void removeEdges(QSet<QUuid>& edgeUuids);

    void moveNodesAction(const QSet<QUuid> &nodes, const QPointF& undoOffset);

    void doMoveNodes(const QSet<QUuid> &nodes, const QPointF& undoOffset);

    void setNodeSize(const QUuid& objectid, const QSizeF& size);

    void setNodeGeometry(const QUuid& objectid, const QRectF& size);

    void setPropertyAction(const QUuid &objectid,
                           const QString& propertyName,
                           const QVariant& oldValue,
                           const QVariant& newValue);

    QSet<Connection> connections(const QSet<QUuid>& edgeUuids) const;

    ObjectGraphNode* node(const QUuid& nodeUuid) const;

    ObjectGraphEdge* edge(const QUuid& edgeUuid) const;


    void contextMenuEvent(QGraphicsSceneContextMenuEvent* e) override;
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

    QMap<QUuid,ObjectGraphNode*> _nodes;
    QMap<QUuid,ObjectGraphEdge*> _edges;

    QUndoStack* _commandStack;

    bool _moveInProgress;
    QPointF _pasteOffset;
};

#endif // NODEGRAPH_H
