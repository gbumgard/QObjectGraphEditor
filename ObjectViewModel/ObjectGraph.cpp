
#include <QApplication>
#include <QGraphicsView>
#include <QGraphicsSceneEvent>
#include <QGraphicsItem>
#include <QWidget>
#include <QMetaMethod>
#include <QDataStream>
#include <QDrag>
#include <QList>
#include <QUndoCommand>
#include <QMap>
#include <QMessageBox>
#include <QStyle>
#include <QIcon>
#include <QClipboard>
#include <QMimeData>
#include <QDebug>

#include "ObjectFactory.h"
#include "ObjectModel.h"
#include "Connection.h"
#include "ObjectGraph.h"
#include "ObjectGraphNode.h"
#include "ObjectGraphEdge.h"
//#include "../QObjectGraphEditorApp/QObjectGraphEditorApp.h"


#include "SlotMimeData.h"
#include "SignalMimeData.h"
#include "GraphicsItemMimeData.h"

#include "AddNodeCommand.h"
#include "AddEdgeCommand.h"
#include "MoveNodesCommand.h"
#include "SetNodeSizeCommand.h"
#include "SetPropertyCommand.h"
#include "CutCommand.h"
#include "PasteCommand.h"
#include "DeleteCommand.h"


/**
 * @brief ObjectGraph::ObjectGraph
 * @param parent
 */
ObjectGraph::ObjectGraph(QObject* parent)
  : QGraphicsScene(parent)
  , _fileName()
  , _moveInProgress(false)
{
  qDebug() << Q_FUNC_INFO;
  setObjectName(metaObject()->className());
}

ObjectGraph::~ObjectGraph() {
  qDebug() << Q_FUNC_INFO;
  clear();
}


/**
 * @brief serialize
 *  Writes a serialized representation of a graph to a data stream.
 *  The specified nodes and any connections between those nodes are serialized.
 *  Any connections to nodes outside of the set are excluded unless the
 *  allConnections flag is set to true.
 * @param out
 *  The data stream that is the destination for the serialized graph.
 * @param nodeUuids
 *  Identifies the nodes in the graph to be serialized.
 * @param allConnections
 *  If true, connections to objects outside of the specified object set are also
 *  serialized, as is required for a reversible delete operation.
 * @return
 */
bool ObjectGraph::serialize(QDataStream &out,
                            QSet<QUuid>& nodeUuids,
                            bool allConnections) const {

  qDebug() << Q_FUNC_INFO;

  if (nodeUuids.empty()) {
    // Serialize all nodes if the set is empty
    for (auto nodeUuid : _nodes.keys()) {
      nodeUuids.insert(nodeUuid);
    }
  }

  qDebug() << "\nNodes to be stored";
  for (auto nodeUuid : nodeUuids) qDebug() << nodeUuid;

  // Nodes are identified by the UUID of the objects they manage.
  // The keys of the map is the list of UUIDs for objects to be serialized.

  if (_model->serialize(out,nodeUuids,allConnections)) {

    QMap<QUuid,QMap<QString,QVariant>> graphProperties;

    for (auto nodeUuid : nodeUuids) {
      ObjectGraphNode* node = _nodes[nodeUuid];
      const QMetaObject* metaObj = node->metaObject();
      for (int i = 0; i < metaObj->propertyCount(); i++) {
        QMetaProperty property = metaObj->property(i);
        if (property.userType() > QMetaType::UnknownType &&
            property.userType() < QMetaType::User &&
            property.isValid() &&
            !property.isConstant() &&
            property.isReadable() &&
            property.isWritable() &&
            property.isStored()) {
          qDebug() << "property"
                   << "name" << property.name()
                   << "type" << property.typeId()
                   << "typeName" << property.typeName()
                   << "value" << property.read(node);
          graphProperties[nodeUuid][property.name()] = property.read(node);
        }
      }
    }

    qDebug() << "\nStored Graph Properties";
    for (auto nodeUuid : graphProperties.keys()) {
      qDebug() << "Stored Node Properties" << nodeUuid;
      for (auto propName : graphProperties[nodeUuid].keys()) {
        qDebug() << "  " << nodeUuid << propName << graphProperties[nodeUuid][propName];
      }
    }

    out << graphProperties;

    return true;

  }

  return false;
}

/**
 * @brief deserialize
 *  Reads a serialized representation of a graph from a data stream and inserts
 *  it into the graphics scene. Used for file-open and various commands including
 *  cut, paste and delete.
 * @param in
 *  The stream from which to read the serialized graph.
 * @param selectNodes
 *  If true, all of the nodes are selected after deserialization, otherwise none are.
 * @param makeUnique
 *  If true, new UUIDs will be assigned to all of the objects and connections as is required
 *  for paste operations.
 * @param placement
 *  Indicates how and whether the position argument is used.
 * @param position
 *  The scene position at which to place the graph (used for paste commands).
 * @param nodeUuids
 *  The UUIDs of all of the objects that were added to the scene will be inserted in this set.
 *  This argument will be changed to identify the centroid position for the nodes that are added.
 * @return
 */
bool ObjectGraph::deserialize(QDataStream& in,
                              bool selectNodes,
                              bool makeUnique,
                              Placement placement,
                              QPointF& position,
                              QSet<QUuid>& nodeUuids) {

  qDebug() << Q_FUNC_INFO;

  QMap<QUuid,QUuid> uuidMap;

  if (!in.atEnd() && _model->deserialize(in, makeUnique, uuidMap) && !in.atEnd()) {

    QMap<QUuid,QMap<QString,QVariant>> graphProperties;

    in >> graphProperties;

    if (makeUnique) {

      // Replace the UUIDs originally assigned to nodes with new object UUIDs

      for (auto oldNodeUuid : graphProperties.keys()) {
        QUuid newNodeUuid = uuidMap[oldNodeUuid];
        if (newNodeUuid != oldNodeUuid) {
          QMap<QString,QVariant> property = graphProperties[oldNodeUuid];
          graphProperties[newNodeUuid] = property;
          graphProperties.remove(oldNodeUuid);
        }
      }
    }

    qDebug() << "\nRetrieved Graph Properties";
    for (auto nodeUuid : graphProperties.keys()) {
      qDebug() << "Retrieved Node Properties" << nodeUuid;
      for (auto name : graphProperties[nodeUuid].keys()) {
        qDebug() << "  " << nodeUuid << name << graphProperties[nodeUuid][name];
      }
    }


    QPointF originalCentroid;
    double zoffset = topNodeZValue();

    for (auto nodeUuid : uuidMap) {
      ObjectGraphNode* node = _nodes[nodeUuid];
      if (node) {
        const QMetaObject* metaObject = node->metaObject();
        for (auto propName : graphProperties[nodeUuid].keys()) {
          int propIndex = metaObject->indexOfProperty(propName.toUtf8());
          if (propIndex != -1) {
            metaObject->property(propIndex).write(node,graphProperties[nodeUuid][propName]);
          }
        }
        node->setSelected(selectNodes);
        node->setZValue(node->zValue()+zoffset);
        originalCentroid += node->scenePos();
      }
      else {
        qWarning() << "ERROR: graph node" << nodeUuid << "not found.";
      }
    }

    originalCentroid /= graphProperties.keys().size();

    if (placement == OriginalPlacement) {

      position = originalCentroid;

    }
    else {

      QPointF finalCentroid;
      QPointF centroidOffset = position - originalCentroid;

      for (auto nodeUuid : uuidMap) {
        ObjectGraphNode* node = _nodes[nodeUuid];
        if (node) {
          if (placement == AbsolutePlacement) {
            QPointF absolutePosition = node->scenePos() + centroidOffset;
            node->setPos(absolutePosition);
            finalCentroid += absolutePosition;
          }
          else if (placement == OffsetPlacement) {
            QPointF offsetPosition = node->scenePos() + position;
            node->moveBy(offsetPosition.x(),offsetPosition.y());
            finalCentroid += offsetPosition;
          }
          node->setSelected(false);
          node->setSelected(selectNodes);
        }
      }

      finalCentroid /= graphProperties.keys().size();
      position = finalCentroid;

    }

    for (auto nodeUuid : uuidMap.values()) {
      nodeUuids.insert(nodeUuid);
    }

    return true;
  }

  return false;
}

void ObjectGraph::clear() {
  qDebug() << Q_FUNC_INFO;
  _commandStack->clear();
  _fileName.clear();
  _model->clear();
}


qreal ObjectGraph::topNodeZValue() {
  static qreal top = 0;
  return top++;
}

qreal ObjectGraph::topEdgeZValue() {
  static qreal top = std::numeric_limits<double>::lowest();
  return top++;
}


void ObjectGraph::setModel(ObjectModel* model) {
  _model = model;
  connect(_model,&ObjectModel::objectAdded,this,&ObjectGraph::onObjectAdded);
  connect(_model,&ObjectModel::objectRemoved,this,&ObjectGraph::onObjectRemoved);
  connect(_model,&ObjectModel::connectionAdded,this,&ObjectGraph::onConnectionAdded);
  connect(_model,&ObjectModel::connectionRemoved,this,&ObjectGraph::onConnectionRemoved);
  connect(_model,&ObjectModel::objectStatusChanged,this,&ObjectGraph::onObjectStatusChanged);
}

ObjectModel* ObjectGraph::model() {
  return _model;
}

QSet<Connection> ObjectGraph::connections(const QSet<QUuid>& edgeUuids) const {
  return _model->connections(edgeUuids);
}

ObjectGraphNode* ObjectGraph::node(const QUuid &nodeUuid) const {
  return _nodes[nodeUuid];
}

ObjectGraphEdge* ObjectGraph::edge(const QUuid &edgeUuid) const {
  return _edges[edgeUuid];
}



void ObjectGraph::addNodeAction(const QString &className, const QPointF& position) {
  qDebug() << Q_FUNC_INFO << className;
  _commandStack->push(new AddNodeCommand(this,className,position));
}

void ObjectGraph::addNode(const QUuid& objectId, const QString &className) {
  qDebug() << Q_FUNC_INFO << objectId << className;
  model()->createObject(className,objectId);
}

void ObjectGraph::removeNodes(QSet<QUuid>& nodeUuids) {
  qDebug() << Q_FUNC_INFO << nodeUuids;
  _model->removeObjects(nodeUuids);
}


void ObjectGraph::addEdgeAction(const QUuid& senderUuid,
                                const QString& signalSignature,
                                const QUuid& receiverUuid,
                                const QString& slotSignature) {

  qDebug() << "\n" << Q_FUNC_INFO;
  qDebug() << "sender:" << senderUuid
           << "signal:" << signalSignature
           << "receiver:" << receiverUuid
           << "slot:" << slotSignature;

  _commandStack->push(new AddEdgeCommand(this,senderUuid,signalSignature,receiverUuid,slotSignature));
}

void ObjectGraph::addEdge(const QUuid &connectionUuid,
                          const QUuid &senderUuid,
                          const QString &signalSignature,
                          const QUuid &receiverUuid,
                          const QString &slotSignature) {

  qDebug() << "\n" << Q_FUNC_INFO;
  qDebug() << "connection:" << connectionUuid
           << "sender:" << senderUuid
           << "signal:" << signalSignature
           << "receiver:" << receiverUuid
           << "slot:" << slotSignature;

  model()->createConnection(connectionUuid, senderUuid, signalSignature, receiverUuid, slotSignature);
}

void ObjectGraph::removeEdges(QSet<QUuid>& edgeUuids) {
  qDebug() << Q_FUNC_INFO << edgeUuids;
  model()->removeConnections(edgeUuids);
}





void ObjectGraph::moveNodesAction(const QSet<QUuid>& nodes, const QPointF& undoOffset) {
  _commandStack->push(new MoveNodesCommand(this, nodes, undoOffset));
}

void ObjectGraph::setNodeSize(const QUuid &objectId, const QSizeF& size) {
  _commandStack->push(new SetNodeSizeCommand(this,objectId,size));
}

void ObjectGraph::setPropertyAction(const QUuid& objectId,
                              const QString& propertyName,
                              const QVariant& prevValue,
                              const QVariant& nextValue) {
  qDebug() << Q_FUNC_INFO << propertyName << prevValue << nextValue;
  _commandStack->push(new SetPropertyCommand(this,objectId,propertyName,prevValue,nextValue));
}


void ObjectGraph::onUndoAction() {
  qDebug() << Q_FUNC_INFO;
  _commandStack->undo();
}

void ObjectGraph::onRedoAction() {
  qDebug() << Q_FUNC_INFO;
  _commandStack->redo();
}

void ObjectGraph::onCutAction() {
  qDebug() << Q_FUNC_INFO;
  onCopyAction();
  onDeleteAction();
}

void ObjectGraph::onCopyAction() {

  qDebug() << Q_FUNC_INFO;

  QByteArray serializedGraph;
  QDataStream out(&serializedGraph,QIODevice::OpenModeFlag::WriteOnly);

  QSet<QUuid> nodeUuids;
  QList<QGraphicsItem*> items = selectedItems();

  for (auto item : items) {
    ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
    if (node) nodeUuids.insert(node->uuid());
  }

  serialize(out,nodeUuids,false);

  QMimeData* mimeData = new QMimeData;
  mimeData->setData(ObjectGraph::SERIALIZED_GRAPH_MIME_TYPE, serializedGraph);
  QApplication::clipboard()->clear();
  QApplication::clipboard()->setMimeData(mimeData);

  qDebug() << "copied nodes" << nodeUuids;
}

void ObjectGraph::onPasteAction() {

  qDebug() << Q_FUNC_INFO;

  static const QPointF offset(16,16);

  //_commandStack->beginMacro("paste from clipboard");
  //_commandStack->push(new DeleteCommand(this));
  _commandStack->push(new PasteCommand(this,OffsetPlacement,_pasteOffset));
  //_commandStack->endMacro();

  _pasteOffset.setX(((long)_pasteOffset.x() + 16) % 128);
  _pasteOffset.setY(((long)_pasteOffset.y() + 16) % 128);

}

void ObjectGraph::onPasteAction(const QPointF& scenePosition) {

  qDebug() << Q_FUNC_INFO;

  _commandStack->beginMacro("paste from clipboard");
  _commandStack->push(new DeleteCommand(this));
  _commandStack->push(new PasteCommand(this,AbsolutePlacement,scenePosition));
  _commandStack->endMacro();

}

void ObjectGraph::onDeleteAction() {
  qDebug() << Q_FUNC_INFO;
  _commandStack->push(new DeleteCommand(this));
}

void ObjectGraph::onSelectAllAction() {
  qDebug() << Q_FUNC_INFO;
  for (auto item : items()) {
    ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
    if (node) node->setSelected(true);
    else {
      ObjectGraphEdge* edge = dynamic_cast<ObjectGraphEdge*>(item);
      if (edge) edge->setSelected(true);
    }
  }
}


void ObjectGraph::onContextHelpAction() {
  qDebug() << Q_FUNC_INFO;
}


void ObjectGraph::onObjectAdded(const QUuid& objectUuid) {

  qDebug() << Q_FUNC_INFO << objectUuid;

  QObject* obj = model()->object(objectUuid);

  ObjectGraphNode* objectNode = new ObjectGraphNode(obj);

  addItem(objectNode);

  _nodes.insert(objectUuid,objectNode);

  clearSelection();

  objectNode->setSelected(true);

  onObjectStatusChanged(objectUuid);
}

void ObjectGraph::onObjectRemoved(const QUuid &objectUuid) {

  qDebug() << Q_FUNC_INFO << objectUuid;

  ObjectGraphNode* node = _nodes.take(objectUuid);

  if (node) {

    removeItem(node);
    node->close();

  }
}

void ObjectGraph::onConnectionAdded(const QUuid& connectionUuid) {

  qDebug() << Q_FUNC_INFO << connectionUuid;

  ObjectGraphEdge* connectionPath = new ObjectGraphEdge(connectionUuid);

  Connection connection = model()->connection(connectionUuid);

  ObjectGraphNode* sender = _nodes[connection.senderUuid()];
  ObjectGraphNode* receiver = _nodes[connection.receiverUuid()];

  sender->bindToSignalConnectionPoint(connection.signalSignature(),connectionPath);
  receiver->bindToSlotConnectionPoint(connection.slotSignature(),connectionPath);

  addItem(connectionPath);

  _edges.insert(connectionUuid,connectionPath);

  clearSelection();

  connectionPath->setSelected(true);
}

void ObjectGraph::onConnectionRemoved(const QUuid &connectionUuid) {
  qDebug() << Q_FUNC_INFO << connectionUuid;
  ObjectGraphEdge* edge = _edges.take(connectionUuid);
  if (edge) {
    removeItem(edge);
    edge->deleteLater();
    //delete edge;
  }
}

void ObjectGraph::onObjectStatusChanged(const QUuid& objectUuid) {
  QObject* obj = model()->object(objectUuid);
  if (obj) {
    ObjectGraphNode* node = _nodes[objectUuid];
    node->setStatus(ObjectModel::getObjectStatusCodeProperty(node->object()),
                    ObjectModel::getObjectStatusMessageProperty(node->object()));
  }
}


void  ObjectGraph::contextMenuEvent(QGraphicsSceneContextMenuEvent* e) {

  qDebug() << Q_FUNC_INFO << e;

  QMenu menu;
  QAction* actionUndo = menu.addAction(QIcon::fromTheme("edit-undo"),"Undo",QKeySequence::Undo,this,&ObjectGraph::onUndoAction);
  QAction* actionRedo = menu.addAction(QIcon::fromTheme("edit-redo"),"Redo",QKeySequence::Redo,this,&ObjectGraph::onRedoAction);
  QAction* actionCut = menu.addAction(QIcon::fromTheme("edit-cut"),"Cut",QKeySequence::Cut,this,&ObjectGraph::onCutAction);
  QAction* actionCopy = menu.addAction(QIcon::fromTheme("edit-copy"),"Copy",QKeySequence::Copy,this,&ObjectGraph::onCopyAction);
  QAction* actionPaste = menu.addAction(QIcon::fromTheme("edit-paste"),"Paste",QKeySequence::Paste,[this,e]() {onPasteAction(e->scenePos());});
  QAction* actionDelete = menu.addAction(QIcon::fromTheme("edit-delete"),"Delete",QKeySequence::Delete,this,&ObjectGraph::onDeleteAction);
  menu.addAction(QIcon::fromTheme("edit-select-all"),"Select All",QKeySequence::SelectAll,this,&ObjectGraph::onSelectAllAction);
  menu.addAction(QIcon::fromTheme("help-contents"),"Context Help",QKeySequence(Qt::Key_F1),this,&ObjectGraph::onContextHelpAction);

  actionUndo->setDisabled(!_commandStack->canUndo());
  actionRedo->setDisabled(!_commandStack->canRedo());

  if (selectedItems().isEmpty()) {
    actionCut->setDisabled(true);
    actionCopy->setDisabled(true);
    actionDelete->setDisabled(true);
  }

  actionPaste->setDisabled(true);
  const QMimeData* mimeData = QApplication::clipboard()->mimeData();
  QStringList formats = mimeData->formats();
  for (auto format : formats) {
    if (format == ObjectGraph::SERIALIZED_GRAPH_MIME_TYPE) {
      actionPaste->setDisabled(false);
      break;
    }
  }

  menu.exec(e->screenPos());

}


void ObjectGraph::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {

  event->ignore();
  QGraphicsScene::mouseMoveEvent(event);

}

void ObjectGraph::mousePressEvent(QGraphicsSceneMouseEvent *event) {

  _moveInProgress = nullptr != itemAt(event->scenePos(),QTransform()) && event->button() == Qt::LeftButton;

  qDebug() << "\nPRESS" << Q_FUNC_INFO << event << "moving:" << _moveInProgress;

  QGraphicsScene::mousePressEvent(event);

}

void ObjectGraph::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {

  qDebug() << "\nRELEASE" << Q_FUNC_INFO << event;

  if (_moveInProgress) {

    _moveInProgress = false;

    QPointF startPosition = event->buttonDownScenePos(Qt::LeftButton);
    QPointF endPosition = event->scenePos();
    QPointF undoOffset = startPosition - endPosition;
    QList<QGraphicsItem*> selection = selectedItems();

    QSet<QUuid> nodeUuids;

    for (QGraphicsItem* item : selection) {
      ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
      if (node) {
        nodeUuids.insert(node->uuid());
      }
    }

    event->accept();

    moveNodesAction(nodeUuids,undoOffset);

  }

\
  QGraphicsScene::mouseReleaseEvent(event);
}

void ObjectGraph::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {

  QGraphicsScene::dragEnterEvent(event);
}

void ObjectGraph::dragLeaveEvent(QGraphicsSceneDragDropEvent *event) {

  QGraphicsScene::dragLeaveEvent(event);
}

void ObjectGraph::dropEvent(QGraphicsSceneDragDropEvent *event) {

  qDebug() << "\n" << Q_FUNC_INFO;

  QGraphicsScene::dropEvent(event);

  const QMimeData* mimeData = event->mimeData();

  if (mimeData->hasFormat(QStringLiteral("application/x-qstandarditemmodeldatalist"))) {
    if (event->dropAction() == Qt::CopyAction) {
      const QMimeData* mimeData = event->mimeData();
      if (mimeData->hasFormat(QStringLiteral("application/x-qstandarditemmodeldatalist"))) {
        QByteArray encoded = mimeData->data(QStringLiteral("application/x-qstandarditemmodeldatalist"));
        QDataStream stream(&encoded, QIODevice::ReadOnly);
        int row, col;
        while (!stream.atEnd()) {
          QMap<int,  QVariant> roleDataMap;
          stream >> row >> col >> roleDataMap;
          if (roleDataMap.contains(Qt::UserRole+1)) {
            QString className = roleDataMap[Qt::UserRole+1].toString();
            addNodeAction(className,event->scenePos());
          }
        }
        event->acceptProposedAction();
      }
    }
  }

  QGraphicsScene::dropEvent(event);

}

void ObjectGraph::dragMoveEvent(QGraphicsSceneDragDropEvent *event) {

  //qDebug() << "\n" << Q_FUNC_INFO;

  const QMimeData* mimeData = event->mimeData();

  if (mimeData->hasFormat(QStringLiteral("application/x-qstandarditemmodeldatalist"))) {

    // Handle object class drop test

    QByteArray encoded = mimeData->data(QStringLiteral("application/x-qstandarditemmodeldatalist"));
    QDataStream stream(&encoded, QIODevice::ReadOnly);
    int row, col;
    while (!stream.atEnd()) {
      QMap<int,  QVariant> roleDataMap;
      stream >> row >> col >> roleDataMap;
    }
    event->accept();

  }
  else if (mimeData->hasFormat(SlotMimeData::MIME_TYPE)) {
    // Handle connection endpoint drag
    const SlotMimeData* slotMimeData = dynamic_cast<const SlotMimeData*>(mimeData);
    if (slotMimeData) {
      ObjectGraphEdge* connectionPath = slotMimeData->edge();
      connectionPath->setSignalPosition(event->scenePos());
    }
    QGraphicsScene::dragMoveEvent(event);
  }
  else if (mimeData->hasFormat(SignalMimeData::MIME_TYPE)) {
    // Handle connection endpoint drag
    const SignalMimeData* signalMimeData = dynamic_cast<const SignalMimeData*>(mimeData);
    if (signalMimeData) {
      ObjectGraphEdge* connectionPath = signalMimeData->edge();
      connectionPath->setSlotPosition(event->scenePos());
    }
    QGraphicsScene::dragMoveEvent(event);
  }
  else if (mimeData->hasFormat(QStringLiteral("application/x-graphicsitem-size"))) {

    const GraphicsItemMimeData* dragMimeData = qobject_cast<const GraphicsItemMimeData*>(event->mimeData());
    if (dragMimeData) {
      ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(dragMimeData->item());

      QRectF geom = node->geometry();
      double targetWidth = event->scenePos().x() - geom.left();
      double targetHeight = event->scenePos().y() - geom.top();
      QSizeF newSize(targetWidth,targetHeight);
      setNodeSize(node->objectUuid(),newSize);
    }
  }
  else {
    QGraphicsScene::dragMoveEvent(event);
  }

}

void ObjectGraph::keyReleaseEvent(QKeyEvent * keyEvent)
{

  qDebug() << "\n" << Q_FUNC_INFO << keyEvent;

  if(keyEvent->key() == Qt::Key_Delete) {
    _commandStack->push(new DeleteCommand(this));
    keyEvent->accept();
  }

  QGraphicsScene::keyReleaseEvent(keyEvent);

}


