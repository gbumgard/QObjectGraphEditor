#include "ObjectModel.h"
#include "ObjectFactory.h"

#include <QMetaObject>
#include <QMetaMethod>
#include <QMetaProperty>

#include <QDataStream>
#include <QUuid>
#include <QFile>

#include <QDebug>
#include <iostream>

QUuid ObjectModel::_uuid("{1bc25d19-117d-427c-bc5f-3019109c681d}");

const char* ObjectModel::ID_PROPERTY_NAME = "object-model-id";
const char* ObjectModel::MODEL_PROPERTY_NAME = "object-model";

int ObjectModel::_nextObjectId = 0;
int ObjectModel::_nextConnectionId = 0;

ObjectModel::ObjectModel(QObject* parent)
  : QObject(parent)
{
}

ObjectModel::~ObjectModel() {
  clear();
}

void ObjectModel::clear() {
  QList<int> objectIds = _objects.keys();
  for (auto objectId : objectIds) {
    removeObject(objectId);
  }
}

bool ObjectModel::load(const QString& fileName) {
  clear();
  QFile file(fileName);
  file.open(QIODevice::ReadOnly);
  QDataStream in(&file);
  return read(in);
}

bool ObjectModel::read(QDataStream &in) {
  QUuid uuid;
  in >> uuid;
  if (uuid == _uuid) {
    qint32 version;
    in >> version;
    QMap<int,QString> objectClassNames;
    in >> objectClassNames;
    for (auto objectId : objectClassNames.keys()) {
      addObject(objectClassNames[objectId],objectId);
    }
    QMap<int,QMap<QString,QVariant>> modelProperties;
    in >> modelProperties;
    for (auto objectId : modelProperties.keys()) {
      if (objectId > _nextObjectId) _nextObjectId = objectId + 1;
      QObject* obj = object(objectId);
      if (obj) {
        const QMetaObject* metaObject = obj->metaObject();
        QMap<QString,QVariant> objectProperties = modelProperties[objectId];
        for (auto propName : objectProperties.keys()) {
          int propIndex;
          if (-1 != (propIndex = metaObject->indexOfProperty(propName.toUtf8()))) {
            metaObject->property(propIndex).write(obj,objectProperties[propName]);
          }
        }
      }
    }
    qDebug() << modelProperties;
    QMap<int,QPair<QPair<int,QString>,QPair<int,QString>>> connections;
    in >> connections;
    for (auto connectionId : connections.keys()) {
      if (connectionId > _nextConnectionId) _nextConnectionId = connectionId + 1;
      QPair<QPair<int,QString>,QPair<int,QString>> connectionDef = connections[connectionId];
      int senderId = connectionDef.first.first;
      QString signalSignature = connectionDef.first.second;
      int receiverId = connectionDef.second.first;
      QString slotSignature = connectionDef.second.second;
      addConnection(senderId,signalSignature,receiverId,slotSignature,connectionId);
    }
    return true;
  }
  return false;
}

bool ObjectModel::save(const QString& fileName) const {
  QFile file(fileName);
  file.open(QIODevice::WriteOnly);
  QDataStream out(&file);
  return write(out);
}

bool ObjectModel::write(QDataStream &out) const {
  // Write a header with a "magic number" and a version
  out << _uuid;
  out << (qint32)CURRENT_VERSION;

  QMap<int,QString> objectClassNames;
  for (auto key : _objects.keys()) {
    objectClassNames.insert(key,_objects[key]->metaObject()->className());
  }
  out << objectClassNames;
  QMap<int,QMap<QString,QVariant>> modelProperties;
  for (auto key : _objects.keys()) {
    QObject* object = _objects[key];
    QMap<QString,QVariant> objectProperties;
    const QMetaObject* metaObject = object->metaObject();
    for (int i = 0; i < metaObject->propertyCount(); i++) {
      QMetaProperty property = metaObject->property(i);
      if (property.isValid() &&
          property.isReadable() &&
          property.isWritable() &&
          property.isStored()) {
        objectProperties.insert(property.name(),property.read(object));
      }
    }
    modelProperties.insert(key,objectProperties);
  }
  qDebug() << modelProperties;
  out << modelProperties;
  QMap<int,QPair<QPair<int,QString>,QPair<int,QString>>> connections;
  for (auto connectionId : _connectionIndex.keys()) {
    Connection connection = _connectionIndex[connectionId];
    QObject* sender = _objects[connection.senderId()];
    QObject* receiver = _objects[connection.receiverId()];
    QString signalSignature = sender->metaObject()->method(connection.signalIndex()).methodSignature();
    QString slotSignature = receiver->metaObject()->method(connection.slotIndex()).methodSignature();
    QPair<int,QString> signalDef(connection.senderId(),signalSignature);
    QPair<int,QString> slotDef(connection.receiverId(),slotSignature);
    QPair<QPair<int,QString>,QPair<int,QString>> connectionDef(signalDef,slotDef);
    connections.insert(connectionId,connectionDef);
  }
  out << connections;
  return true;
}

void ObjectModel::dump() const {
  std::cout << std::endl << "### Object Model ###" << std::endl;
  for (auto key : _objects.keys()) {
    std::cout << "object-id=" << key << " class-name=" << _objects[key]->metaObject()->className() << std::endl;
  }
  for (auto key : _connectionIndex.keys()) {
    Connection connection = _connectionIndex[key];
    std::cout << "connection-id=" << key
              << " sender-id=" << connection.senderId()
              << " signal-index=" << connection.signalIndex()
              << " receiver-id=" << connection.receiverId()
              << " slot-index=" << connection.slotIndex()
              << std::endl;
  }
}

QObject* ObjectModel::object(int objectId) const {
  return _objects.value(objectId);
}

/**
 * @brief ObjectModel::getObjectId
 * Retrieves the ID of an object given a pointer to the object.
 * @param object
 * @return The object ID or -1 if the object is not contained in the model.
 */
int ObjectModel::getObjectId(const QObject* object) const {
  qDebug() << Q_FUNC_INFO;
  int objectId = -1;
  if (object) {
    QVariant v = object->property(ID_PROPERTY_NAME);
    if (v.isValid() && v.type() == QVariant::Int) {
      int id = v.toInt();
      if (_objects.contains(id))
        objectId = id;
    }
    else {
      for (auto key : _objects.keys()) {
        if (_objects[key] == object) {
          objectId = key;
          break;
        }
      }
    }
  }
  return objectId;
}

/**
 * @brief ObjectModel::className
 * Convenience method for obtaining the class name of an object.
 * @param objectId
 * @return
 */
QString ObjectModel::className(int objectId) const {
  QObject* obj = object(objectId);
  QString className;
  if (obj) {
    className = obj->metaObject()->className();
  }
  return className;
}

/**
 * @brief ObjectModel::containsObject
 * Indicates whether an object with the specified ID exists within the model.
 * @param objectId
 * @return
 */
bool ObjectModel::containsObject(int objectId) const {
  return _objects.contains(objectId);
}

/**
 * @brief ObjectModel::containsObject
 * Indicates whether the specified object exists within the model.
 * @param object
 * @return
 */
bool ObjectModel::containsObject(QObject* object) const {
  return getObjectId(object) != -1;
}

/**
 * @brief ObjectModel::containsConnection
 * Indicates whether a connection with the specified ID exists in the model.
 * @param connectionId
 * @return
 */
bool ObjectModel::containsConnection(int connectionId) const {
  return _connectionIndex.contains(connectionId);
}

/**
 * @brief ObjectModel::connection
 * Returns the connection with the specified ID.
 * @param connectionId
 * @return
 */
Connection ObjectModel::connection(int connectionId) const {
  if (_connectionIndex.contains(connectionId)) {
    return _connectionIndex.value(connectionId);
  }
  return Connection();
}

/**
 * @brief ObjectModel::connections
 * Returns a list of all connections contained in the model.
 * @return
 */
QList<Connection> ObjectModel::connections() const {
  return _connectionIndex.values();
}

/**
 * @brief ObjectModel::connections
 * Returns a list of all connections associated with an object.
 * @param objectId
 * @return
 */
QList<Connection> ObjectModel::connections(int objectId) const {
  QList<Connection> list;
  if (_objectConnectionIndex.contains(objectId)) {
    QMap<int,QSet<int>> methodConnectionIndex = _objectConnectionIndex[objectId];
    QList<int> connectionIds;
    for (auto methodConnections : methodConnectionIndex) {
      connectionIds.append(methodConnections.toList());
    }
    for (auto connectionId : connectionIds) {
      list.append(_connectionIndex[connectionId]);
    }
  }
  return list;
}

/**
 * @brief ObjectModel::connections
 * Returns a list of all connections associated with a method on an object.
 * The method index may identify a signal or a slot.
 * @param objectId The object ID
 * @param methodIndex The ::QMetaObject method index for the signal or slot.
 * @return
 */
QList<Connection> ObjectModel::connections(int objectId, int methodIndex) const {
  QList<Connection> list;
  if (_objectConnectionIndex.contains(objectId) &&
      _objectConnectionIndex[objectId].contains(methodIndex)) {
    QList<int> connectionIds = _objectConnectionIndex[objectId][methodIndex].toList();
    for (auto connectionId : connectionIds) {
      list.append(_connectionIndex[connectionId]);
    }
  }
  return list;
}

/**
 * @brief ObjectModel::canConnect
 * Tests whether two methods can be connected as a signal-slot pair.
 * @param senderId The object ID of the signal sender.
 * @param signalIndex The ::QMetaObject method index for the signal.
 * @param receiverId The object ID of the slot receiver.
 * @param slotIndex The ::QMetaObject method index for the slot.
 * @return Boolean value indicating whether the signal and slot methods can be connected.
 */
bool ObjectModel::canConnect(int senderId,
                             int signalIndex,
                             int receiverId,
                             int slotIndex) const {
  bool result = false;
  QObject* sender = object(senderId);
  if (sender) {
    QObject* receiver = object(receiverId);
    if (receiver) {
      const QMetaObject* senderMetaObject = sender->metaObject();
      const QMetaObject* receiverMetaObject = receiver->metaObject();
      if (signalIndex >=0 && signalIndex < senderMetaObject->methodCount()) {
        if (slotIndex >=0 && slotIndex < receiverMetaObject->methodCount()) {
          result = QMetaObject::checkConnectArgs(senderMetaObject->method(signalIndex),
                                                 receiverMetaObject->method(slotIndex));
        }
      }
    }
  }
  return result;
}

/**
 * @brief ObjectModel::addObject
 * Adds a new object to the model. The className parameter must identify a
 * an ::QObject class that has been registered with the ::ObjectFactory.
 * This method will return -1 if the specified class has not been registered with the
 * factory or if an object with the specified ID already exists and that object is an
 * instance of another class.
 * @param className The name of a class registered with #ObjectFactory.
 * @param objectId The object ID if reconstituting an object otherwise -1 for a new object.
 * @return The ID of the object or -1 if an object of the specified type could not be added.
 */
int ObjectModel::addObject(const QString &className, int objectId) {

  if (objectId == -1 || !containsObject(objectId)) {

    QObject* object = ObjectFactory::create(className);
    if (object) {

      // Object ID of -1 indicates new object else reconstitute object with specified ID.
      objectId = (objectId == -1 ? _nextObjectId++ : objectId);

      object->setProperty(ID_PROPERTY_NAME,objectId);
      object->setProperty(MODEL_PROPERTY_NAME,QVariant().fromValue<ObjectModel*>(this));
      _objects.insert(objectId,object);
      emit objectAdded(objectId);
    }
  }
  else {
    // Object with specified ID already ObjectViewModel - check for exact match.
    qWarning() << "object " << objectId << " already exists!";
    if (object(objectId)->metaObject()->className() != className)
      // Pre-existing object does not match object type to be added - indicate failure.
      objectId = -1;
  }

  return objectId;
}

bool ObjectModel::removeObject(int objectId) {
  qDebug() << Q_FUNC_INFO << objectId;
  bool result = false;
  if (_objects.contains(objectId)) {

    if (_objectConnectionIndex.contains(objectId)) {
      QMap<int,QSet<int>> methodConnectionIndex = _objectConnectionIndex.take(objectId);
      for (auto methodConnectionSet : methodConnectionIndex) {
        for (auto connectionId : methodConnectionSet) {
          qDebug() << "remove connection" << connectionId;
          removeConnection(connectionId);
        }
      }
    }
    QObject* obj = _objects.take(objectId);
    if (obj) {
      qDebug() << "delete object" << objectId;
      obj->setParent(nullptr);
      obj->deleteLater();
    }
    result = true;
    emit objectRemoved(objectId);
  }
  return result;
}

/**
 * \internal
 * @brief ObjectModel::addConnection
 * Adds a connection using normalized method signatures to identify the signal and slot methods.
 * @param senderId
 * @param signalSignature
 * @param receiverId
 * @param slotSignature
 * @param connectionId
 * @return
 */
int ObjectModel::addConnection(int senderId,
                               const QString& signalSignature,
                               int receiverId,
                               const QString& slotSignature,
                               int connectionId) {

  const QMetaObject* senderMetaObject = object(senderId)->metaObject();
  const QMetaObject* receiverMetaObject = object(receiverId)->metaObject();

  int signalIndex = senderMetaObject->indexOfSignal(signalSignature.toUtf8().constData());
  if (signalIndex != -1) {
    int slotIndex = receiverMetaObject->indexOfSlot(slotSignature.toUtf8().constData());
    if (slotIndex != -1) {
      return addConnection(senderId,signalIndex,receiverId,slotIndex,connectionId);
    }
  }
  return -1;
}

int ObjectModel::addConnection(int senderId,
                               int signalIndex,
                               int receiverId,
                               int slotIndex,
                               int connectionId) {
  if (connectionId == -1 || !containsConnection(connectionId)) {
    if (connect(senderId,signalIndex,receiverId,slotIndex)) {

      // Connection ID of -1 indicates new connection else reconstitute connection with specified ID.
      connectionId = (connectionId == -1 ? _nextConnectionId++ : connectionId);

      _connectionIndex.insert(connectionId,Connection(senderId,signalIndex,receiverId,slotIndex,connectionId));
      if (!_objectConnectionIndex.contains(senderId)) {
        _objectConnectionIndex.insert(senderId,QMap<int,QSet<int>>());
      }
      else if (!_objectConnectionIndex[senderId].contains(signalIndex)) {
        _objectConnectionIndex[senderId].insert(signalIndex,QSet<int>());
      }
      _objectConnectionIndex[senderId][signalIndex].insert(connectionId);

      if (!_objectConnectionIndex.contains(receiverId)) {
        _objectConnectionIndex.insert(receiverId,QMap<int,QSet<int>>());
      }
      else if (!_objectConnectionIndex[receiverId].contains(slotIndex)) {
        _objectConnectionIndex[receiverId].insert(slotIndex,QSet<int>());
      }
      _objectConnectionIndex[receiverId][slotIndex].insert(connectionId);

      emit connectionAdded(connectionId);
    }
  }
  else {
    // Connection with specified ID already exists - check for exact match.
    Connection conn = connection(connectionId);
    if (conn.senderId() != senderId ||
        conn.receiverId() != receiverId ||
        conn.signalIndex() != signalIndex ||
        conn.slotIndex() != slotIndex)
      // Pre-existing connection does not match connection to be added - indicate failure.
      connectionId = -1;
  }
  return connectionId;
}

bool ObjectModel::removeConnection(int connectionId) {
  qDebug() << Q_FUNC_INFO << connectionId;
  bool result = false;
  if (_connectionIndex.contains(connectionId)) {
    Connection connection = _connectionIndex.take(connectionId);
    int senderId = connection.senderId();
    int signalIndex = connection.signalIndex();
    int receiverId = connection.receiverId();
    int slotIndex = connection.slotIndex();
    disconnect(senderId,signalIndex,receiverId,slotIndex); // may already be disconnected?
    _objectConnectionIndex[senderId][signalIndex].remove(connectionId);
    _objectConnectionIndex[receiverId][slotIndex].remove(connectionId);
    emit connectionRemoved(connectionId);
    result = true;
  }
  return result;
}

bool ObjectModel::connect(int senderId, int signalIndex, int receiverId, int slotIndex) const {
  bool result = false;
  QObject* sender = object(senderId);
  if (sender) {
    QObject* receiver = object(receiverId);
    if (receiver) {
      const QMetaObject* senderMetaObject = sender->metaObject();
      const QMetaObject* receiverMetaObject = receiver->metaObject();
      if (signalIndex >=0 && signalIndex < senderMetaObject->methodCount()) {
        if (slotIndex >=0 && slotIndex < receiverMetaObject->methodCount()) {
          result = QMetaObject::checkConnectArgs(senderMetaObject->method(signalIndex),
                                                 receiverMetaObject->method(slotIndex));
          if (result) {
            result = QObject::connect(sender,senderMetaObject->method(signalIndex),
                                      receiver,receiverMetaObject->method(slotIndex));
          }
        }
      }
    }
  }
  return result;
}

bool ObjectModel::disconnect(int senderId, int signalIndex, int receiverId, int slotIndex) const {
  bool result = false;
  QObject* sender = object(senderId);
  if (sender) {
    QObject* receiver = object(receiverId);
    if (receiver) {
      const QMetaObject* senderMetaObject = sender->metaObject();
      const QMetaObject* receiverMetaObject = receiver->metaObject();
      result = QObject::disconnect(sender,senderMetaObject->method(signalIndex),
                                   receiver,receiverMetaObject->method(slotIndex));
    }
  }
  return result;
}
