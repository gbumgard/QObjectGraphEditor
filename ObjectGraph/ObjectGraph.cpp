#include "ObjectGraph.h"
#include "ObjectFactory.h"
#include "ObjectGraphDatabase.h"

#include <QDir>
#include <QPluginLoader>
#include <QMetaObject>
#include <memory>

/*
int _objectIndex;

static ObjectFactory _objectFactory;

QMap<QString,ObjectFactory*> _objectFactoryPluginClasses;

QMap<QString,QMetaObject*> _objectTypes;

QMap<QString,QMetaObject*> _converterObjectTypes;

QMap<QString,QObject*> _objectInstances;

QMap<QPair<QObject*,int>,QPair<QObject*,int>> _slotConnections;

QMap<QPair<QObject*,int>,QList<QPair<QObject*,int>>> _signalConnections;
*/

std::unique_ptr<ObjectGraph> ObjectGraph::constructObjectGraph(QObject* parent) {
  return std::unique_ptr<ObjectGraph>(new ObjectGraph(parent));
}

ObjectGraph::ObjectGraph(QObject *parent)
  : QObject(parent)
  , _objectIndex(0)
{

}

bool ObjectGraph::loadFromDatabase(const QString &dbFileName,
                                   const QString &sqlDriverName) {
  return _database.open(dbFileName,sqlDriverName);
}


#if 0
bool ObjectGraph::save(const QString& path);
bool ObjectGraph::restore(const QString& path);
#endif

/*
bool ObjectGraph::addObjectType(const QMetaObject& metaObject) {
  return ObjectFactory::addObjectType(metaObject);
}
*/
bool ObjectGraph::containsObjectClass(const QString& className) const {
  return ObjectFactory::contains(className);
}

bool ObjectGraph::containsObjectClass(const QMetaObject& metaObject) const {
  return ObjectFactory::contains(metaObject.className());
}
/*
bool ObjectGraph::removeObjectType(const QString& className) {
  return ObjectFactory::removeObjectType(className);
}

bool ObjectGraph::removeObjectType(const QMetaObject& metaObject) {
  return ObjectFactory::removeObjectType(metaObject.className());
}
*/
QMetaObject ObjectGraph::getMetaObject(const QString& className) const {
  return ObjectFactory::getObjectType(className);
}


int ObjectGraph::createNode(const QString& className) {
  if (containsObjectClass(className)) {
    int nodeId = _objectIndex++;
    std::unique_ptr<QObject> object(ObjectFactory::create(className));
    if (object) {
      QString name = className+QString("#")+QString("%1").arg(nodeId);
      object->setObjectName(name);
      if (_database.insertNode(nodeId,object.get())) {
        _objectInstances[nodeId] = std::move(object);
        return nodeId;
      }
    }
  }
  return -1;
}

QObject* ObjectGraph::getNode(int nodeId) {
  return _objectInstances.at(nodeId).get();
}

QObject* ObjectGraph::getNode(const QString& objectName) {
  (void)objectName;
  return nullptr;
}

bool ObjectGraph::containsNode(int nodeId){
  (void)nodeId;
  return false;
}

bool ObjectGraph::containsNode(const QString& objectName) const {
  (void)objectName;
  return false;
}

bool ObjectGraph::containsNode(const QObject* object) const {
  (void)object;
  return false;
}

bool ObjectGraph::removeNode(int nodeId) {
  (void)nodeId;
  return false;
}

bool ObjectGraph::removeNode(const QString& objectName) {
  (void)objectName;
  return false;
}

bool ObjectGraph::removeNode(const QObject* object) {
  (void)object;
  return false;
}

#if 0

bool ObjectGraph::containsConverterType(const QString& signalSignature, const QString& slotSignature) const;
QString ObjectGraph::getConverterType(const QString& signalSignature, const QString& slotSignature) const;

QList<QString> ObjectGraph::getObjectsBySlotSignature(const QString& slotSignature, bool connected = false);

bool ObjectGraph::canConnect(const QString& signalSignature, const QString& slotSignature) const;

bool ObjectGraph::addConnection(const QString& senderObjectName, const QMetaMethod& signalMethod,
                   const QString& receiverObjectName, const QMetaMethod& slotMethod);
bool ObjectGraph::addConnection(QObject* senderObject, const QMetaMethod& signalMethod,
                   QObject* receiverObject, const QMetaMethod& slotMethod);
bool ObjectGraph::addConnection(const QString& senderObjectName, const QString& signalSignature,
                   const QString& receiverObjectName, const QString& slotSignature);
bool ObjectGraph::addConnection(const QString& senderObjectName, int signalIndex,
                   const QString& receiverObjectName, int slotIndex);
bool ObjectGraph::addConnection(QObject* senderObject, const QString& signalSignature,
                   QObject* receiverObject, const QString& slotSignature);
bool ObjectGraph::addConnection(QObject* senderObject, int signalIndex,
                   QObject* receiverObject, int slotIndex);

bool ObjectGraph::removeConnection(const QString& senderObjectName, const QMetaMethod& signalMethod,
                      const QString& receiverObjectName, const QMetaMethod& slotMethod);
bool ObjectGraph::removeConnection(const QString& senderObjectName, const QString& signalSignature,
                      const QString& receiverObjectName, const QString& slotSignature);
bool ObjectGraph::removeConnection(const QString& senderObjectName, int signalIndex,
                      const QString& receiverObjectName, int slotIndex);

bool ObjectGraph::removeSignalConnections(const QString& senderObjectName, const QString& signalSignature);
bool ObjectGraph::removeSignalConnections(const QString& senderObjectName, int signalIndex);

bool ObjectGraph::removeSignalConnection(const QString& senderObjectName,
                            const QString& signalSignature,
                            const QString receiverObjectName);

bool ObjectGraph::removeSignalConnection(QObject* senderObject,
                            int signalIndex,
                            QObject* receiverObject);

bool ObjectGraph::removeSlotConnection(const QString& receiverObjectName, const QString& slotSignature);
bool ObjectGraph::removeSlotConnection(const QString& receiverObjectName, int slotIndex);
bool ObjectGraph::removeSlotConnection(QObject* receiverObject, const QString& slotSignature);
bool ObjectGraph::removeSlotConnection(QObject* receiverObject, int slotIndex);

bool ObjectGraph::isSignalConnected(const QString& senderObjectName, const QString& signalSignature) const;
bool ObjectGraph::isSignalConnected(const QString* senderObjectName, int signalIndex) const;

bool ObjectGraph::isSlotConnected(const QString& receiverObjectName, const QString& slotSignature) const;
bool ObjectGraph::isSlotConnected(const QString& receiverObjectName, int slotIndex) const;

QList<QPair<QString,QString>> ObjectGraph::signalConnectionsBySignature(const QString& senderObjectName, const QString& signalSignature) const;
QList<QPair<QObject*,QString>> ObjectGraph::signalConnectionsBySignature(const QObject* senderObject, const QString& signalSignature) const;
QList<QPair<QString,int>> ObjectGraph::signalConnectionsByIndex(const QString& senderObjectName, int signalIndex) const;
QList<QPair<QObject*,int>> ObjectGraph::signalConnectionsByIndex(const QObject* senderObject, int signalIndex) const;

QPair<QString,QString> ObjectGraph::slotConnectionBySignature(const QString& receiverObjectName, const QString& slotSignature) const;
QPair<QObject*,QString> ObjectGraph::slotConnectionBySignature(const QObject* receiverObject, const QString& slotSignature) const;
QPair<QString,int> ObjectGraph::slotConnectionsByIndex(const QString& receiverObjectName, int slotIndex) const;
QPair<QObject*,int> ObjectGraph::slotConnectionsByIndex(const QObject* receiverObject, int slotIndex) const;
#endif
