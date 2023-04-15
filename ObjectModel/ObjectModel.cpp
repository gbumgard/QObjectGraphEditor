#include "ObjectModel.h"
#include "ObjectFactory.h"

#include <QMetaObject>
#include <QMetaMethod>
#include <QMetaProperty>

#include <QDataStream>
#include <QUuid>
#include <QFile>
#include <QWidget>

#include <QDebug>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <list>

QUuid ObjectModel::_uuid("{1bc25d19-117d-427c-bc5f-3019109c681d}");

ObjectModel::ObjectModel(QObject* parent)
  : QObject(parent)
{
  setObjectName(metaObject()->className());
}

ObjectModel::~ObjectModel() {
  clear();
}

void ObjectModel::clear() {

  qDebug() << Q_FUNC_INFO;

  for (auto connectionUuid : _connectionIndex.keys()) {
    removeConnection(connectionUuid);
  }

  for (auto objectUuid : _objectIndex.keys()) {
    removeObject(objectUuid);
  }
}

/**
 * @brief serialize Writes a serialized representaion of an object graph to a stream.
 * The objectUUids parameter can be used to specify a subset of the objects
 * contained in the graph to support cut, copy, paste and delete operations.
 * @param out
 * @param objectUuidSet The objects to serialize. Updated to reflect those that actually were.
 * @param allConnections If true, all connections to the specified objects
 * are serialized, otherwise, only those between the objectin the set are.
 * @return
 */
bool ObjectModel::serialize(QDataStream &out,
                            QSet<QUuid>& objectUuidSet,
                            bool allConnections) const {

  qDebug() << Q_FUNC_INFO;

  QSet<QUuid> connectionUuidSet;

  if (!objectUuidSet.empty()) {

    // The object set contains object UUIDs. Serialize those objects only.

    // Find all the connections on objects in the set

    if (allConnections) {

      // Get the UUIDs for all connections on objects in the set.

      for (auto objectUuid : objectUuidSet) {
        connectionUuidSet += connectionUuids(objectUuid);
      }

    }
    else {

      // Get the UUIDs for all connections between objects in the set.

      for (auto connectionUuid : _connectionIndex.keys()) {

        Connection connection = _connectionIndex[connectionUuid];

        if (objectUuidSet.contains(connection.senderUuid()) &&
            objectUuidSet.contains(connection.receiverUuid())) {

          // This connection links two objects in the set
          connectionUuidSet.insert(connectionUuid);

        }
      }
    }
  }
  else {

    // The object set contains no object UUIDS - serialize all of them.

    // Add all of the objects and connections to the sets.

    for (auto objectUuid : _objectIndex.keys()) {
      objectUuidSet.insert(objectUuid);
    }

    for (auto connectionUuid : _connectionIndex.keys()) {
      connectionUuidSet.insert(connectionUuid);
    }
  }

  // Map object UUIDs to class names

  QMap<QUuid,QString> objectClassNames;

  for (auto objectUuid : objectUuidSet) {
    objectClassNames.insert(objectUuid,_objectIndex[objectUuid]->metaObject()->className());
  }

  // Map object UUIDs to properties

  QMap<QUuid,QMap<QString,QVariant>> modelProperties;

  for (auto objectUuid : objectUuidSet) {
    QObject* object = _objectIndex[objectUuid];
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
    modelProperties.insert(objectUuid,objectProperties);
  }

  // Map connection UUIDs to sender and receiver UUIDS and signal/slot signatures

  QMap<QUuid,QPair<QPair<QUuid,QString>,QPair<QUuid,QString>>> connections;

  for (auto connectionUuid : connectionUuidSet) {

    Connection connection = _connectionIndex[connectionUuid];
    QString signalSignature = connection.signalSignature();
    QString slotSignature = connection.slotSignature();

    QPair<QUuid,QString> signalDef(connection.senderUuid(),signalSignature);
    QPair<QUuid,QString> slotDef(connection.receiverUuid(),slotSignature);

    QPair<QPair<QUuid,QString>,QPair<QUuid,QString>> connectionDef(signalDef,slotDef);
    connections.insert(connectionUuid,connectionDef);
  }


  qDebug() << "\nStored Class Names";
  for (auto objectUuid : objectClassNames.keys()) qDebug() << "  " << objectUuid << objectClassNames[objectUuid];

  qDebug() << "\nStored Object Model Properties";
  for (auto objectUuid : modelProperties.keys()) {
    qDebug() << "Stored Object Properties" << objectUuid;
    for (auto name : modelProperties[objectUuid].keys()) {
      qDebug() << "  " << objectUuid << name << modelProperties[objectUuid][name];
    }
  }

  qDebug() << "\nStored Connections";
  for (auto connection : connections) qDebug() << "  " << connection;


  // Write the mappings to the data stream

  out << objectClassNames;
  out << modelProperties;
  out << connections;

  return true;
}


/**
 * @brief deserialize Reads and inserts a serialized represention of an object graph from a stream.
 * @param in The data stream from which to read the serialized representation.
 * @param makeUnique If true, all objects and connections are assigned new UUIDs.
 * @param uuidMap A mapping between original UUIDs and new UUIDs.
 * @return
 */
bool ObjectModel::deserialize(QDataStream &in, bool makeUnique, QMap<QUuid, QUuid>& uuidMap) {

  qDebug() << Q_FUNC_INFO;

  if (!in.atEnd()) {

    QMap<QUuid,QString> objectClassNames;
    QMap<QUuid,QMap<QString,QVariant>> modelProperties;
    QMap<QUuid,QPair<QPair<QUuid,QString>,QPair<QUuid,QString>>> connections;

    in >> objectClassNames;
    in >> modelProperties;
    in >> connections;

    if (makeUnique) {

      for (auto objectUuid : objectClassNames.keys()) uuidMap[objectUuid] = QUuid::createUuid();

      for (auto connectionUuid : connections.keys()) uuidMap[connectionUuid] = QUuid::createUuid();

      for (auto objectUuid : objectClassNames.keys()) {
        objectClassNames[uuidMap[objectUuid]] = objectClassNames[objectUuid];
        objectClassNames.remove(objectUuid);
      }

      for (auto objectUuid : modelProperties.keys()) {
        modelProperties[uuidMap[objectUuid]] = modelProperties[objectUuid];
        modelProperties.remove(objectUuid);
      }

      for (auto connectionUuid : connections.keys()) {
        QPair<QPair<QUuid,QString>,QPair<QUuid,QString>> connection = connections[connectionUuid];
        QPair<QUuid,QString> signalEndpoint = connection.first;
        QPair<QUuid,QString> slotEndpoint = connection.second;
        signalEndpoint.first = uuidMap[signalEndpoint.first];
        slotEndpoint.first = uuidMap[slotEndpoint.first];
        connections[uuidMap[connectionUuid]] = QPair<QPair<QUuid,QString>,QPair<QUuid,QString>>(signalEndpoint,slotEndpoint);
        connections.remove(connectionUuid);
      }
    }

    for (auto objectUuid : objectClassNames.keys()) {
      createObject(objectClassNames[objectUuid],objectUuid);
    }

    for (auto objectUuid : modelProperties.keys()) {
      QObject* obj = object(objectUuid);
      if (obj) {
        const QMetaObject* metaObject = obj->metaObject();
        for (auto propName : modelProperties[objectUuid].keys()) {
          int propIndex = metaObject->indexOfProperty(propName.toUtf8());
          if (propIndex != -1) {
            metaObject->property(propIndex).write(obj,modelProperties[objectUuid][propName]);
          }
        }
      }
    }

    for (auto connectionUuid : connections.keys()) {
      QPair<QPair<QUuid,QString>,QPair<QUuid,QString>> connectionDef = connections[connectionUuid];
      QUuid senderUuid = connectionDef.first.first;
      QString signalSignature = connectionDef.first.second;
      QUuid receiverUuid = connectionDef.second.first;
      QString slotSignature = connectionDef.second.second;
      createConnection(connectionUuid,senderUuid,signalSignature,receiverUuid,slotSignature);
    }


    qDebug() << "\nSRetrieved Objects";
    QMapIterator<QUuid,QString> names(objectClassNames);
    while (names.hasNext()) {
      names.next();
      qDebug() << "  " << names.key();
    }

    qDebug() << "\nSRetrieved Object Model Properties";
    QMapIterator<QUuid,QMap<QString,QVariant>> props(modelProperties);
    while (props.hasNext()) {
      props.next();
      qDebug() << "Stored Object Properties" << props.key();
      QMapIterator<QString,QVariant> objProps(modelProperties[props.key()]);
      while (objProps.hasNext()) {
        objProps.next();
        qDebug() << "  " << props.key() << objProps.key() << modelProperties[props.key()][objProps.key()];
      }
    }

    qDebug() << "\nRetrieved Connections";
    for (auto connection : connections) qDebug() << "  " <<  connection;

    qDebug() << Q_FUNC_INFO;
    topologicalSort();
    return true;
  }

  return false;
}


void ObjectModel::dump() const {
  QMapIterator<QUuid,QObject*> objs(_objectIndex);
  while (objs.hasNext()) {
    objs.next();
    qDebug() << "object-id=" << objs.key() << " class-name=" << _objectIndex[objs.key()]->metaObject()->className();
  }

  QMapIterator<QUuid,Connection> conns(_connectionIndex);
  while (conns.hasNext()) {
    conns.next();
    Connection connection = _connectionIndex[conns.key()];
    qDebug()  << "connection-id=" << conns.key()
             << " sender-id=" << connection.senderUuid()
             << " signal-index=" << connection.signalSignature()
             << " receiver-id=" << connection.receiverUuid()
             << " slot-index=" << connection.slotSignature();
  }
}


/*****************************************************************************************/


void ObjectModel::createObject(const QString &className, const QUuid& objectUuid) {

  if (!_objectIndex.contains(objectUuid)) {
    QObject* object = ObjectFactory::create(className);
    if (object) {
      object->setProperty(OBJECT_ID_PROPERTY_NAME,objectUuid);
      object->setProperty(OBJECT_MODEL_PROPERTY_NAME,QVariant::fromValue<ObjectModel*>(this));
      _objectIndex.insert(objectUuid,object);
      emit objectAdded(objectUuid);
    }
    qDebug() << Q_FUNC_INFO;
    for(auto& conn : _connectionIndex) {
      qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
    }
    topologicalSort();
  }
  else {
    qWarning() << "object" << objectUuid << "already exists!";
  }

}

bool ObjectModel::removeObject(const QUuid &objectUuid) {

  if (_objectIndex.contains(objectUuid)) {

    if (_objectConnectionIndex.contains(objectUuid)) {
      QSet<QUuid> connectionUuids;
      QMap<QString,QSet<QUuid>> methodConnectionIndex = _objectConnectionIndex.take(objectUuid);
      for (auto& methodConnectionSet : methodConnectionIndex) {
        for (auto connectionUuid : methodConnectionSet) {
          qDebug() << "remove connection" << connectionUuid;
          connectionUuids.insert(connectionUuid);
        }
      }
      if (!connectionUuids.empty()) {
        removeConnections(connectionUuids);
      }
      for(auto& conn : _connectionIndex) {
        qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
      }
      qDebug() << Q_FUNC_INFO;
      for(auto& conn : _connectionIndex) {
        qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
      }
      topologicalSort();

    }

    QObject* obj = _objectIndex.take(objectUuid);

    if (obj) {
      qDebug() << "delete object" << objectUuid;
      obj->setParent(nullptr);
      obj->deleteLater();
#if 0
      if (dynamic_cast<QWidget*>(obj)) {
        obj->deleteLater();
      }
      else {
        delete obj;
      }
#endif
    }

    emit objectRemoved(objectUuid);

    return true;
  }

  return false;

}

bool ObjectModel::removeObjects(QSet<QUuid>& objectUuids) {

  qDebug() << Q_FUNC_INFO << objectUuids;

  QSet<QUuid> removedUuids;

  for (auto objectUuid : objectUuids) {

    if (removeObject(objectUuid)) {
      removedUuids.insert(objectUuid);
    }

  }

  // Return the set of objects that were actually removed.
  objectUuids = removedUuids;

  qDebug() << Q_FUNC_INFO;
  for(auto& conn : _connectionIndex) {
    qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
  }
  topologicalSort();

  return true;
}

bool ObjectModel::containsObject(const QObject* const object) const {
  return !objectUuid(object).isNull();
}

QObject* ObjectModel::object(const QUuid& objectUuid) const {
  return _objectIndex.value(objectUuid);
}

/**
 * @brief ObjectModel::getObjectId
 * Retrieves the ID of an object given a pointer to the object.
 * @param object
 * @return The object ID or -1 if the object is not contained in the model.
 */
QUuid ObjectModel::objectUuid(const QObject* const object) const {
  QUuid objectId = QUuid();
  if (object) {
    QVariant v = object->property(OBJECT_ID_PROPERTY_NAME);
    if (v.isValid() && v.typeId() == QMetaType::QUuid) {
      QUuid id = v.toUuid();
      if (_objectIndex.contains(id))
        objectId = id;
    }
    else {
      for (auto key : _objectIndex.keys()) {
        if (_objectIndex[key] == object) {
          objectId = key;
          break;
        }
      }
    }
  }
  return objectId;
}

QSet<QUuid> ObjectModel::objectUuids() const {
  return QSet<QUuid>(_objectIndex.keys().begin(),_objectIndex.keys().end());
}


QString ObjectModel::objectClassName(const QUuid& objectUuid) const {
  QObject* obj = object(objectUuid);
  QString className;
  if (obj) {
    className = obj->metaObject()->className();
  }
  return className;
}


/*****************************************************************************************/


void ObjectModel::createConnection(const QUuid& connectionUuid,
                                   const QUuid& senderUuid,
                                   const QString& signalSignature,
                                   const QUuid& receiverUuid,
                                   const QString& slotSignature) {

  qDebug() << Q_FUNC_INFO;
  qDebug() << "connection:" << connectionUuid
           << "sender:" << senderUuid
           << "signal:" << signalSignature
           << "receiver:" << receiverUuid
           << "slot:" << slotSignature;

  if (connect(senderUuid,signalSignature,receiverUuid,slotSignature)) {

    _connectionIndex.insert(connectionUuid,Connection(senderUuid,signalSignature,receiverUuid,slotSignature,connectionUuid));

    // Add any necessary signal map and or connection set required to complete the sender entry.
    if (!_objectConnectionIndex.contains(senderUuid)) {
      _objectConnectionIndex.insert(senderUuid,QMap<QString,QSet<QUuid>>());
    }
    else if (!_objectConnectionIndex[senderUuid].contains(signalSignature)) {
      _objectConnectionIndex[senderUuid].insert(signalSignature,QSet<QUuid>());
    }

    // Add the connection UUID to the sender->signal->connection map.
    _objectConnectionIndex[senderUuid][signalSignature].insert(connectionUuid);

    // Add any necessary slot map and or connection set required to complete the receiver entry.
    if (!_objectConnectionIndex.contains(receiverUuid)) {
      _objectConnectionIndex.insert(receiverUuid,QMap<QString,QSet<QUuid>>());
    }
    else if (!_objectConnectionIndex[receiverUuid].contains(slotSignature)) {
      _objectConnectionIndex[receiverUuid].insert(slotSignature,QSet<QUuid>());
    }

    // Add the connection UUID to the receiver->slot->connection map.
    _objectConnectionIndex[receiverUuid][slotSignature].insert(connectionUuid);

    qDebug() << Q_FUNC_INFO;
    for(auto& conn : _connectionIndex) {
      qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
    }
    topologicalSort();

    emit connectionAdded(connectionUuid);
  }
}

bool ObjectModel::removeConnection(const QUuid &connectionUuid) {

  qDebug() << Q_FUNC_INFO << connectionUuid;

  if (_connectionIndex.contains(connectionUuid)) {

    Connection connection = _connectionIndex.take(connectionUuid);

    QUuid senderId = connection.senderUuid();
    QString signalSignature = connection.signalSignature();
    QUuid receiverId = connection.receiverUuid();
    QString slotSignature = connection.slotSignature();

    disconnect(senderId,signalSignature,receiverId,slotSignature); // may already be disconnected?

    _objectConnectionIndex[senderId][signalSignature].remove(connectionUuid);
    _objectConnectionIndex[receiverId][slotSignature].remove(connectionUuid);

    qDebug() << Q_FUNC_INFO;
    for(auto& conn : _connectionIndex) {
      qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
    }
    topologicalSort();

    emit connectionRemoved(connectionUuid);

    return true;
  }
  else {
    qDebug() << "cannot remove connection" << connectionUuid << "because it does not exist.";
  }

  return false;
}

bool ObjectModel::removeConnections(QSet<QUuid>& connectionUuids) {

  qDebug() << Q_FUNC_INFO << connectionUuids;

  QSet<QUuid> removedUuids;

  for (auto connectionUuid : connectionUuids) {
    if (removeConnection(connectionUuid)) {
      removedUuids.insert(connectionUuid);
    }
  }

  // Return the set of connections actually removed.
  connectionUuids = removedUuids;
  qDebug() << Q_FUNC_INFO;
  for(auto& conn : _connectionIndex) {
    qDebug() << "conn: " << conn.connectionUuid() << "sender:" << conn.senderUuid() << "[" << conn.signalSignature() << "] receiver:" << conn.receiverUuid()  << "[" << conn.slotSignature() << "]";
  }
  topologicalSort();

  return true;
}

bool ObjectModel::containsConnection(const Connection& connection) const {
  return _connectionIndex.contains(connection.connectionUuid());
}

Connection ObjectModel::connection(const QUuid& connectionUuid) const {
  if (_connectionIndex.contains(connectionUuid)) {
    return _connectionIndex.value(connectionUuid);
  }
  return Connection();
}

QSet<Connection> ObjectModel::connections(const QSet<QUuid>& connectionUuids) const {
  QSet<Connection> connections;
  for (auto connectionUuid : connectionUuids) {
    connections.insert(_connectionIndex[connectionUuid]);
  }
  return connections;
}

QSet<QUuid> ObjectModel::connectionUuids() const {
  return QSet<QUuid>(_connectionIndex.keys().begin(),_connectionIndex.keys().end());
}

QSet<QUuid> ObjectModel::connectionUuids(const QUuid& objectUuid) const {
  QSet<QUuid> connectionUuids;
  if (_objectConnectionIndex.contains(objectUuid)) {
    QMap<QString,QSet<QUuid>> methodConnectionIndex = _objectConnectionIndex[objectUuid];
    for (auto methodConnections : methodConnectionIndex.values()) {
      connectionUuids += methodConnections;
    }
    for (auto connectionUuid : connectionUuids) {
      connectionUuids.insert(connectionUuid);
    }
  }
  return connectionUuids;
}

QSet<QUuid> ObjectModel::connectionUuids(const QUuid& objectId, const QString& methodSignature) const {
  QSet<QUuid> connectionUuids;
  if (_objectConnectionIndex.contains(objectId) &&
      _objectConnectionIndex[objectId].contains(methodSignature)) {
    connectionUuids = QSet<QUuid>(_objectConnectionIndex[objectId][methodSignature]);
  }
  return connectionUuids;
}


/*****************************************************************************************/


bool ObjectModel::canConnect(const QUuid& senderUuid,
                             const QString& signalSignature,
                             const QUuid& receiverUuid,
                             const QString& slotSignature) const {
  bool result = false;

  QObject* sender = object(senderUuid);

  if (sender) {

    QObject* receiver = object(receiverUuid);

    if (receiver) {

      const QMetaObject* senderMetaObject = sender->metaObject();
      const QMetaObject* receiverMetaObject = receiver->metaObject();

      // Check that the signal is valid
      int signalIndex = senderMetaObject->indexOfSignal(signalSignature.toUtf8());
      if (signalIndex != -1) {

        // Check that the slot is valid
        int slotIndex = receiverMetaObject->indexOfSlot(slotSignature.toUtf8());
        if (slotIndex != -1) {

          // Get the QMetaMethod objects for the test targets
          QMetaMethod signalMetaMethod = senderMetaObject->method(signalIndex);
          QMetaMethod slotMetaMethod = receiverMetaObject->method(slotIndex);

          qDebug() << "checking compatibility of "
                   << senderMetaObject->className() << signalMetaMethod.methodSignature() << "[" << signalMetaMethod.tag() << "]"
                   << "and"
                   << receiverMetaObject->className() << slotMetaMethod.methodSignature() << "[" << slotMetaMethod.tag() << "]";

          if (QMetaObject::checkConnectArgs(signalMetaMethod,slotMetaMethod)) {

            // Are we testing QVariant aruments?
            if ((slotMetaMethod.parameterCount()==1 && slotMetaMethod.parameterType(0) == QMetaType::QVariant) &&
                (signalMetaMethod.parameterCount()==1 && signalMetaMethod.parameterType(0) == QMetaType::QVariant)) {

              // Check for QVariant type restriction tags on the signal and slot methods

              std::string slotTag(slotMetaMethod.tag());

              if (!slotTag.empty()) {

                // The slot imposes type constraints on a QVariant argument so the signal must also.

                std::string signalTag(signalMetaMethod.tag());

                if (signalTag.empty()) {

                  // Cannot connect the signal to the slot because the receiver has specified
                  // constraints on the slot's QVariant parameter using QVARIANT_PAYLOAD tags but
                  // no tags were attached to the signal method declaration.

                  result = false;

                }
                else {

                  // Tokenize the tag strings

                  std::string token;

                  std::vector<std::string> slotTypeRestrictions;
                  std::stringstream slotTagTokenizer(slotTag);

                  while(std::getline(slotTagTokenizer,token,' ')) {
                      int typeId = QMetaType::fromName(token).id();
                    if (typeId != QMetaType::UnknownType) {
                      slotTypeRestrictions.push_back(token);
                    }
                    else {
                      qDebug() << "tag on" << receiverMetaObject->className() << slotMetaMethod.methodSignature()
                               << "specifies unrecognized type" << token.c_str();
                    }
                  }

                  std::vector<std::string> signalTypeRestrictions;
                  std::stringstream signalTagTokenizer(signalTag);

                  while(std::getline(signalTagTokenizer,token,' ')) {
                    int typeId = QMetaType::fromName(token).id();
                    if (typeId != QMetaType::UnknownType) {
                      signalTypeRestrictions.push_back(token);
                    }
                    else {
                      qDebug() << "tag on" << senderMetaObject->className() << signalMetaMethod.methodSignature()
                               << "specifies unrecognized type" << token.c_str();
                    }
                  }

                  // Sort the type lists prior to finding the intersection between the two.

                  std::sort(slotTypeRestrictions.begin(), slotTypeRestrictions.end());
                  std::sort(signalTypeRestrictions.begin(), signalTypeRestrictions.end());

                  // Find intersection between the signal ans slot contraints to see if any match.

                  std::vector<std::string> accepts;
                  std::set_intersection(signalTypeRestrictions.begin(),signalTypeRestrictions.end(),
                                        slotTypeRestrictions.begin(),slotTypeRestrictions.end(),
                                        std::back_inserter(accepts));

                  if (accepts.empty()) {

                    qDebug() << "slot" << receiverMetaObject->className() << slotMetaMethod.methodSignature() << "[" << slotTag.c_str() << "]"
                             << "does not accept QVariant data type" << signalTag.c_str()
                             << "from signal" << senderMetaObject->className() << signalMetaMethod.methodSignature() << "[" << signalTag.c_str() << "]";

                    // Indicate that the signal cannot be connected to the slot
                    // because the advertised QVariant payload types do not match
                    return false;

                  }
                  else {
                    qDebug() << "slot" << receiverMetaObject->className() << slotMetaMethod.methodSignature()
                             << "accepts the following QVariant data types from"
                             << "signal" << senderMetaObject->className() << signalMetaMethod.methodSignature();
                    for (auto s : accepts) {
                      qDebug() << "  -" << s.c_str();
                    }
                  }
                }
              }
            }

            // If we made it here, the signal and slot can be connected
            return true;

          }
        }
      }
    }
  }
  return result;
}


/**
 * @brief ObjectModel::connect
 * @param senderUuid
 * @param signalSignature
 * @param receiverUuid
 * @param slotSignature
 * @return
 */
bool ObjectModel::connect(const QUuid& senderUuid,
                          const QString& signalSignature,
                          const QUuid& receiverUuid,
                          const QString& slotSignature) const {

  qDebug() << "\n" << Q_FUNC_INFO;
  qDebug() << "sender:" << senderUuid
           << "signal:" << signalSignature
           << "receiver:" << receiverUuid
           << "slot:" << slotSignature;

  bool result = false;

  // Check that the receiver and sender both exist

  QObject* receiver = object(receiverUuid);
  QObject* sender = object(senderUuid);

  if (sender && receiver) {

    const QMetaObject* senderMetaObject = sender->metaObject();
    const QMetaObject* receiverMetaObject = receiver->metaObject();

    // Check that the signal and slot is valid
    int signalIndex = senderMetaObject->indexOfSignal(signalSignature.toUtf8());

    if (signalIndex != -1) {

      int slotIndex = receiverMetaObject->indexOfSlot(slotSignature.toUtf8());

      if (slotIndex != -1) {

        result = QMetaObject::checkConnectArgs(senderMetaObject->method(signalIndex),
                                                 receiverMetaObject->method(slotIndex));

        if (result) {
            result = QObject::connect(sender,
                                      senderMetaObject->method(signalIndex),
                                      receiver,
                                      receiverMetaObject->method(slotIndex),
                                      Qt::QueuedConnection);
        }
        else {
            qWarning() << "ERROR! signal" << signalSignature << "cannot be connected to slot" << slotSignature;
        }
      }
      else {
        qWarning() << "ERROR! slot" << slotSignature << "does not exist";
      }
    }
    else {
      qWarning() << "ERROR! signal" << signalSignature << "does not exist";
    }
  }
  else {

    qWarning() << "ERROR! one or both connection endpoints do not exist";

    if (!sender) qWarning() << "sender" << senderUuid << "signal" << signalSignature << "does not exist";
    if (!receiver) qWarning() << "receiver" << receiverUuid << "slot" << slotSignature << "does not exist";

  }
  return result;
}


/**
 * @brief ObjectModel::disconnect
 * @param senderUuid
 * @param signalSignature
 * @param receiverUuid
 * @param slotSignature
 * @return
 */
bool ObjectModel::disconnect(const QUuid& senderUuid,
                             const QString& signalSignature,
                             const QUuid& receiverUuid,
                             const QString& slotSignature) const {
  bool result = false;

  QObject* sender = object(senderUuid);
  QObject* receiver = object(receiverUuid);

  if (sender && receiver) {

    const QMetaObject* senderMetaObject = sender->metaObject();
    const QMetaObject* receiverMetaObject = receiver->metaObject();

    int signalIndex = senderMetaObject->indexOfSignal(signalSignature.toUtf8());
    int slotIndex = receiverMetaObject->indexOfSlot(slotSignature.toUtf8());
    if (signalIndex != -1 && slotIndex != -1) {

      result = QObject::disconnect(sender,senderMetaObject->method(signalIndex),
                                   receiver,receiverMetaObject->method(slotIndex));

    }
    else {
      qWarning() << "ERROR! one or both connection endpoints do not exist";
      if (signalIndex == -1) qWarning() << "sender" << senderMetaObject->className() << senderUuid
                                        << "signal" << signalSignature << "does not exist";
      if (slotIndex == -1) qWarning() << "receiver" << receiverMetaObject->className() << receiverUuid
                                      << "slot" << slotSignature << "does not exist";
    }
  }
  else {
    qWarning() << "ERROR! one or both connection endpoints do not exist";
    if (!sender) qWarning() << "sender" << senderUuid << "signal" << signalSignature << "does not exist";
    if (!receiver) qWarning() << "receiver" << receiverUuid << "slot" << slotSignature << "does not exist";
  }

  return result;
}



/*****************************************************************************************/


/**
 * @brief ObjectModel::getObjectIdProperty
 * @param obj
 * @return
 */
QUuid ObjectModel::getObjectIdProperty(const QObject* obj) {
  QVariant v = obj->property(ObjectModel::OBJECT_ID_PROPERTY_NAME);
  if (v.isValid() && (v.typeId() == QMetaType::QUuid)) {
    return v.toUuid();
  }
  return QUuid();
}


/**
 * @brief ObjectModel::setObjectIdProperty
 * @param obj
 * @param objectUuid
 */
void ObjectModel::setObjectIdProperty(QObject* obj, const QUuid& objectUuid) {
  obj->setProperty(ObjectModel::OBJECT_ID_PROPERTY_NAME,objectUuid);
}


/**
 * @brief ObjectModel::getObjectModelProperty
 * @param obj
 * @return
 */
ObjectModel* ObjectModel::getObjectModelProperty(const QObject* obj) {
  QVariant v = obj->property(ObjectModel::OBJECT_MODEL_PROPERTY_NAME);
  if (v.isValid()) {
    return qvariant_cast<ObjectModel*>(v);
  }
  return nullptr;
}


/**
 * @brief ObjectModel::setObjectModelProperty
 * @param obj
 * @param model
 */
void ObjectModel::setObjectModelProperty(QObject* obj, const ObjectModel* model) {
  obj->setProperty(OBJECT_MODEL_PROPERTY_NAME,QVariant::fromValue(model));
}


/**
 * @brief ObjectModel::setObjectStatusCodeProperty
 * @param obj
 * @param statusCode
 */
void ObjectModel::setObjectStatusCodeProperty(QObject* obj, StatusCode statusCode) {
  obj->setProperty(STATUS_CODE_PROPERTY_NAME,statusCode);
}


/**
 * @brief ObjectModel::getObjectStatusCodeProperty
 * @param obj
 * @return
 */
ObjectModel::StatusCode ObjectModel::getObjectStatusCodeProperty(const QObject* obj) {
  QVariant v = obj->property(ObjectModel::STATUS_CODE_PROPERTY_NAME);
  if (v.isValid()) {
    return (StatusCode)v.toInt();
  }
  return STATUS_UNDEFINED;
}


/**
 * @brief ObjectModel::setObjectStatusMessageProperty
 * @param obj
 * @param statusMessage
 */
void ObjectModel::setObjectStatusMessageProperty(QObject* obj, const QString& statusMessage) {
  obj->setProperty(STATUS_MESSAGE_PROPERTY_NAME,statusMessage);
}


/**
 * @brief ObjectModel::getObjectStatusMessageProperty
 * @param obj
 * @return
 */
QString ObjectModel::getObjectStatusMessageProperty(const QObject* obj) {
  QVariant v = obj->property(ObjectModel::STATUS_MESSAGE_PROPERTY_NAME);
  if (v.isValid()) {
    return v.toString();
  }
  return QString();
}


/**
 * @brief ObjectModel::setObjectStatus
 * @param obj
 * @param statusCode
 * @param statusMessage
 */
void ObjectModel::setObjectStatus(QObject* obj, StatusCode statusCode, const QString& statusMessage) {
  if (statusCode != getObjectStatusCodeProperty(obj) ||
      statusMessage != getObjectStatusMessageProperty(obj)) {
    setObjectStatusCodeProperty(obj,statusCode);
    setObjectStatusMessageProperty(obj,statusMessage);
    updateObjectStatus(obj);
  }
}


/**
 * @brief ObjectModel::updateObjectStatus
 * @param obj
 */
void ObjectModel::updateObjectStatus(QObject* obj) {
  QUuid objectId = getObjectIdProperty(obj);
  if (!objectId.isNull()) {
    ObjectModel* model = getObjectModelProperty(obj);
    if (model) {
      emit model->objectStatusChanged(objectId);
    }
    else {
      qWarning() << Q_FUNC_INFO << "object model property is missing on object" << objectId;
    }
  }
  else {
    qWarning() << Q_FUNC_INFO << "object id property is missing on object" << obj->objectName();
  }
}

void ObjectModel::topologicalSort(const QUuid& senderUuid, std::list<QUuid>& stack, QMap<QUuid,bool>& visited) const {
    if (!visited[senderUuid]) {
        visited[senderUuid] = true;
        auto methodConnectionIndex = _objectConnectionIndex[senderUuid];
        for (auto& methodConnections : methodConnectionIndex) {
          for (auto& connectionUuid : methodConnections) {
            auto receiverUuid = _connectionIndex[connectionUuid].receiverUuid();
            topologicalSort(receiverUuid,stack,visited);
          }
        }
        stack.push_front(senderUuid);
    }
}

std::list<QUuid> ObjectModel::topologicalSort() const {
  std::list<QUuid> stack;
  QMap<QUuid,bool> visited;

  QMapIterator<QUuid,QObject*> objs(_objectIndex);
  while (objs.hasNext()) {
    objs.next();
    topologicalSort(objs.key(),stack,visited);
  }

  int i = 0;
  qDebug() << "object update order:";
  for(auto objectUuid : stack) {
    qDebug() << i++ << ")" << objectUuid << object(objectUuid)->metaObject()->className();
  }
  return stack;
}

