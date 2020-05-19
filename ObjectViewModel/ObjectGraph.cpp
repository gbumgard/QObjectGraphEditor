
#include <QApplication>
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

#include "ObjectFactory.h"
#include "ObjectModel.h"
#include "Connection.h"

#include "ObjectGraph.h"
#include "ObjectGraphNode.h"
#include "ObjectGraphEdge.h"

#include "SlotMimeData.h"
#include "SignalMimeData.h"
#include "GraphicsItemMimeData.h"

#include <QDebug>

enum CommandIds {
  AddNodeCommandId,
  RemoveNodeCommandId,
  SetNodePositionCommandId,
  SetNodeSizeCommandId,
  AddEdgeCommandId,
  RemoveEdgeCommandId,
  SetPropertyCommandId
};

/**
 * \internal
 * @brief The AddNodeCommand class
 */
class AddNodeCommand : public QUndoCommand
{

public:

  AddNodeCommand(ObjectGraph* graph,
                 const QString& className,
                 const QPointF& position,
                 QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _objectId(-1)
    , _className(className)
    , _position(position)
  {
  }

  int id() const override { return AddNodeCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    _objectId = _graph->doAddNode(_objectId, _className);
    if (_objectId == -1) setObsolete(true);
    else {
      ObjectGraphNode* node = _graph->node(_objectId);
      if (node)
        node->setPos(_position);
    }
  }

  /**
   * @brief undo
   */
  void undo() override {
    _graph->doRemoveNode(_objectId);
    if (_objectId == -1) setObsolete(true);
  }

private:

  ObjectGraph* _graph;
  int _objectId;
  QString _className;
  QPointF _position;

};

/**
 * \internal
 * @brief The RemoveNodeCommand class
 */
class RemoveNodeCommand : public QUndoCommand
{

public:

  RemoveNodeCommand(ObjectGraph* graph,
                    int objectId,
                    QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _objectId(objectId)
    , _className()
    , _position()
  {
  }

  int id() const override { return RemoveNodeCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    ObjectGraphNode* node= _graph->node(_objectId);
    if (node) {
      _position = node->scenePos();
      QObject* obj = node->object();
      _className = obj->metaObject()->className();
      _graph->doRemoveNode(_objectId);
      if (_objectId == -1) setObsolete(true);
    }
    else {
      setObsolete(true);
    }
  }

  /**
   * @brief undo
   */
  void undo() override {
    _objectId = _graph->doAddNode(_objectId, _className);
    if (_objectId == -1) setObsolete(true);
    else {
      ObjectGraphNode* node = _graph->node(_objectId);
      node->setPos(_position);
    }
  }

private:

  ObjectGraph* _graph;
  int _objectId;
  QString _className;
  QPointF _position;

};

/**
 * \internal
 * @brief The MoveNodesCommand class
 */
class MoveNodesCommand : public QUndoCommand
{

public:

  /**
   * @brief MoveNodesCommand
   * @param graph
   * @param objectId
   * @param scenePos
   * @param parent
   */
  MoveNodesCommand(ObjectGraph* graph,
                   const QList<int>& nodes,
                   const QPointF& undoOffset,
                   QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _nodes(nodes)
    , _undoOffset(undoOffset)
    , _redoOffset(0,0)
  {
  }

  int id() const override { return SetNodePositionCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    for (auto objectId : _nodes) {
      ObjectGraphNode* node = _graph->node(objectId);
      if (node) {
        node->moveBy(_redoOffset.x(),_redoOffset.y());
      }
    }
  }

  /**
   * @brief undo
   */
  void undo() override {
    for (auto objectId : _nodes) {
      ObjectGraphNode* node = _graph->node(objectId);
      if (node) {
        node->moveBy(_undoOffset.x(),_undoOffset.y());
      }
    }
    _redoOffset = QPointF(-_undoOffset.x(),-_undoOffset.y());
  }

  bool mergeWith(const QUndoCommand* other) override {
    if (other->id() != id()) return false;
    const MoveNodesCommand* cmd = static_cast<const MoveNodesCommand*>(other);
    if (cmd->_nodes != _nodes) return false;
    _undoOffset = cmd->_undoOffset + _undoOffset;
    return true;
  }

protected:

  ObjectGraph* _graph;
  QList<int> _nodes;
  QPointF _undoOffset;
  QPointF _redoOffset;

};

/**
 * \internal
 * @brief The SetNodeSizeCommand class
 */
class SetNodeSizeCommand : public QUndoCommand
{

public:

  /**
   * @brief SetNodeSizeCommand
   * @param graph
   * @param objectId
   * @param size
   * @param parent
   */
  SetNodeSizeCommand(ObjectGraph* graph,
                     int objectId,
                     const QSizeF& newSize,
                     QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _objectId(objectId)
    , _oldSize(newSize)
    , _newSize(newSize)
  {
  }

  int id() const override { return SetNodeSizeCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    ObjectGraphNode* node = _graph->node(_objectId);
    if (node) {
      _oldSize = node->size();
      node->resize(_newSize);
    }
    else {
      setObsolete(true);
    }
  }

  /**
   * @brief undo
   */
  void undo() override {
    ObjectGraphNode* node = _graph->node(_objectId);
    if (node) {
      node->resize(_oldSize);
    }
    else {
      setObsolete(true);
    }
  }

  bool mergeWith(const QUndoCommand* other) override {
    if (other->id() != id()) return false;
    const SetNodeSizeCommand* cmd = static_cast<const SetNodeSizeCommand*>(other);
    if (cmd->_objectId != _objectId) return false;
    _newSize = cmd->_newSize;
    return true;
  }

private:

  ObjectGraph* _graph;
  int _objectId;
  QSizeF _oldSize;
  QSizeF _newSize;

};

/**
 * \internal
 * @brief The AddEdgeCommand class
 */
class AddEdgeCommand : public QUndoCommand
{

public:

  /**
   * @brief AddEdgeCommand
   * @param graph
   * @param senderId
   * @param signalIndex
   * @param receiverId
   * @param slotIndex
   * @param parent
   */
  AddEdgeCommand(ObjectGraph* graph,
                 int senderId,
                 int signalIndex,
                 int receiverId,
                 int slotIndex,
                 QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _connectionId(-1)
    , _senderId(senderId)
    , _signalIndex(signalIndex)
    , _receiverId(receiverId)
    , _slotIndex(slotIndex)
  {
  }

  int id() const override { return AddEdgeCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    _connectionId = _graph->doAddEdge(_connectionId,_senderId,_signalIndex,_receiverId,_slotIndex);
    if (_connectionId == -1) setObsolete(true);
  }

  /**
   * @brief undo
   */
  void undo() override {
    _graph->doRemoveEdge(_connectionId);
    if (_connectionId == -1) setObsolete(true);
  }

private:

  ObjectGraph* _graph;
  int _connectionId;
  int _senderId;
  int _signalIndex;
  int _receiverId;
  int _slotIndex;

};

/**
 * \internal
 * @brief The RemoveEdgeCommand class
 */
class RemoveEdgeCommand : public QUndoCommand
{

public:

  /**
   * @brief RemoveEdgeCommand
   * @param graph
   * @param connectionId
   * @param parent
   */
  RemoveEdgeCommand(ObjectGraph* graph,
                    int connectionId,
                    QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _connectionId(connectionId)
    , _connection()
  {
  }

  int id() const override { return RemoveEdgeCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    _connection = _graph->model()->connection(_connectionId);
    _graph->doRemoveEdge(_connectionId);
    if (_connectionId == -1) setObsolete(true);
  }

  /**
   * @brief undo
   */
  void undo() override {
    _connectionId = _graph->doAddEdge(_connectionId,
                                      _connection.senderId(),
                                      _connection.signalIndex(),
                                      _connection.receiverId(),
                                      _connection.slotIndex());
    if (_connectionId == -1) setObsolete(true);
  }

private:

  ObjectGraph* _graph;
  int _connectionId;
  Connection _connection;

};

/**
 * \internal
 * @brief The SetPropertyCommand class
 */
class SetPropertyCommand : public QUndoCommand
{

public:

  /**
   * @brief SetPropertyCommand
   * @param graph
   * @param objectId
   * @param propertyName
   * @param prevValue
   * @param nextValue
   * @param parent
   */
  SetPropertyCommand(ObjectGraph* graph,
                     int objectId,
                     const QString& propertyName,
                     const QVariant& prevValue,
                     const QVariant& nextValue,
                     QUndoCommand* parent = nullptr)
    : QUndoCommand(parent)
    , _graph(graph)
    , _objectId(objectId)
    , _propertyName(propertyName)
    , _prevValue(prevValue)
    , _nextValue(nextValue)
  {
  }

  int id() const override { return SetPropertyCommandId; }

  /**
   * @brief redo
   */
  void redo() override {
    QObject* obj = _graph->model()->object(_objectId);
    if (obj) {
      obj->setProperty(_propertyName.toUtf8().constData(),_nextValue);
    }
    else {
      setObsolete(true);
    }
  }

  /**
   * @brief undo
   */
  void undo() override {
    QObject* obj = _graph->model()->object(_objectId);
    if (obj) {
      obj->setProperty(_propertyName.toUtf8().constData(),_prevValue);
    }
    else {
      setObsolete(true);
    }
  }

  bool mergeWith(const QUndoCommand* other) override {
    if (other->id() != id()) return false;
    const SetPropertyCommand* cmd = static_cast<const SetPropertyCommand*>(other);
    if (cmd->_objectId != _objectId) return false;
    _nextValue = cmd->_nextValue;
    return true;
  }

private:

  ObjectGraph* _graph;
  int _objectId;
  QString _propertyName;
  QVariant _prevValue;
  QVariant _nextValue;

};


/**
 * @brief ObjectGraph::ObjectGraph
 * @param parent
 */
ObjectGraph::ObjectGraph(QObject* parent)
  : QGraphicsScene(parent)
  , _fileName()
{
}

void ObjectGraph::clear() {
  _fileName.clear();
  _model->clear();
}

bool ObjectGraph::read(QDataStream &in) {
  bool success = true;
  if ((success = _model->read(in))) {
    int objectId;
    while(!in.atEnd() && success) {
      in >> objectId;
      if (objectId == -1) break;
      if (_nodes.contains(objectId)) {
        success = _nodes[objectId]->read(in);
      }
    }
  }
  return success;
}

bool ObjectGraph::write(QDataStream &out) const {
  bool success = true;
  if ((success = _model->write(out))) {
    for (auto objectId : _nodes.keys()) {
      out << objectId;
      if (!(success = _nodes[objectId]->write(out))) break;
    }
    out << -1;
  }
  return success;
}

qreal ObjectGraph::topZValue() {
  static qreal top = 0;
  return top++;
}

ObjectModel* ObjectGraph::model() {
  return _model;
}


void ObjectGraph::setModel(ObjectModel* model) {
  _model = model;
  connect(_model,&ObjectModel::objectAdded,this,&ObjectGraph::onObjectAdded);
  connect(_model,&ObjectModel::objectRemoved,this,&ObjectGraph::onObjectRemoved);
  connect(_model,&ObjectModel::connectionAdded,this,&ObjectGraph::onConnectionAdded);
  connect(_model,&ObjectModel::connectionRemoved,this,&ObjectGraph::onConnectionRemoved);
  // TODO: clear graph and create new one. All nodes will be positioned at 0,0.
}

void ObjectGraph::addNode(const QString &className, const QPointF& position) {
  _commandStack->push(new AddNodeCommand(this,className,position));
}

int ObjectGraph::doAddNode(int objectId, const QString &className) {
  return model()->addObject(className,objectId);
}

void ObjectGraph::removeNode(int objectId) {
  _commandStack->push(new RemoveNodeCommand(this,objectId));
}

bool ObjectGraph::doRemoveNode(int objectId) {
  return model()->removeObject(objectId);
}

void ObjectGraph::moveNodes(const QList<int>& nodes, const QPointF& undoOffset) {
  _commandStack->push(new MoveNodesCommand(this, nodes, undoOffset));
}

void ObjectGraph::setNodeSize(int objectId, const QSizeF& size) {
  _commandStack->push(new SetNodeSizeCommand(this,objectId,size));
}

void ObjectGraph::addEdge(int senderId, int signalIndex, int receiverId, int slotIndex) {
  _commandStack->push(new AddEdgeCommand(this,senderId,signalIndex,receiverId,slotIndex));
}

int ObjectGraph::doAddEdge(int connectionId, int senderId, int signalIndex, int receiverId, int slotIndex) {
  return model()->addConnection(senderId,signalIndex,receiverId,slotIndex,connectionId);
}

void ObjectGraph::removeEdge(int connectionId) {
  _commandStack->push(new RemoveEdgeCommand(this,connectionId));
}

bool ObjectGraph::doRemoveEdge(int connectionId) {
  return model()->removeConnection(connectionId);
}

void ObjectGraph::setProperty(int objectId,
                             const QString& propertyName,
                             const QVariant& prevValue,
                             const QVariant& nextValue) {
  _commandStack->push(new SetPropertyCommand(this,objectId,propertyName,prevValue,nextValue));
}

ObjectGraphNode* ObjectGraph::node(int objectId) const {
  return _nodes[objectId];
}

ObjectGraphEdge* ObjectGraph::edge(int connectionId) const {
  return _edges[connectionId];
}

void ObjectGraph::onObjectAdded(int objectId) {
  QObject* obj = model()->object(objectId);
  //ObjectGraphNode* objectNode = new ObjectGraphNode(obj);
  ObjectGraphNode* objectNode = new ObjectGraphNode(obj);
  addItem(objectNode);
  _nodes.insert(objectId,objectNode);
  clearSelection();
}

void ObjectGraph::onObjectRemoved(int objectId) {
  ObjectGraphNode* node = _nodes.take(objectId);
  if (node) {
    removeItem(node);
    node->close();
  }
}

void ObjectGraph::onConnectionAdded(int connectionId) {
  ObjectGraphEdge* connectionPath = new ObjectGraphEdge(connectionId);
  Connection connection = model()->connection(connectionId);
  ObjectGraphNode* sender = _nodes[connection.senderId()];
  ObjectGraphNode* receiver = _nodes[connection.receiverId()];
  sender->bindToSignalConnectionPoint(connection.signalIndex(),connectionPath);
  receiver->bindToSlotConnectionPoint(connection.slotIndex(),connectionPath);
  addItem(connectionPath);
  _edges.insert(connectionId,connectionPath);
  clearSelection();
}

void ObjectGraph::onConnectionRemoved(int connectionId) {
  ObjectGraphEdge* edge = _edges.take(connectionId);
  if (edge) {
    removeItem(edge);
    edge->deleteLater();
  }
}

void ObjectGraph::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  event->ignore();
  QGraphicsScene::mouseMoveEvent(event);
}

void ObjectGraph::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  QGraphicsScene::mousePressEvent(event);
}

void ObjectGraph::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  QPointF startPosition = event->buttonDownScenePos(Qt::LeftButton);
  QPointF endPosition = event->scenePos();
  QPointF undoOffset = startPosition - endPosition;
  QList<QGraphicsItem*> selection = selectedItems();
  QList<int> nodes;
  for (QGraphicsItem* item : selection) {
    ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
    if (node) {
      nodes.append(node->objectId());
    }
  }
  moveNodes(nodes,undoOffset);
  QGraphicsScene::mouseReleaseEvent(event);
}

void ObjectGraph::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
  QGraphicsScene::dragEnterEvent(event);
}

void ObjectGraph::dragLeaveEvent(QGraphicsSceneDragDropEvent *event) {
  QGraphicsScene::dragLeaveEvent(event);
}

void ObjectGraph::dropEvent(QGraphicsSceneDragDropEvent *event) {

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
            addNode(className,event->scenePos());
            event->acceptProposedAction();
          }
        }
      }
    }
  }
}

void ObjectGraph::dragMoveEvent(QGraphicsSceneDragDropEvent *event) {

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
      setNodeSize(node->objectId(),newSize);
    }
  }
  else {
    QGraphicsScene::dragMoveEvent(event);
  }

}

void ObjectGraph::keyReleaseEvent(QKeyEvent * keyEvent)
{
  if(/*keyEvent->key() == Qt::Key_Backspace || */keyEvent->key() == Qt::Key_Delete) {
    QList<QGraphicsItem*> selection = selectedItems();
    for (QGraphicsItem* item : selection) {
      ObjectGraphNode* node = dynamic_cast<ObjectGraphNode*>(item);
      if (node) {
        removeNode(node->objectId());
      }
      else {
        ObjectGraphEdge* edge = dynamic_cast<ObjectGraphEdge*>(item);
        if (edge) {
          removeEdge(edge->connectionId());
        }
      }
    }
  }
}


