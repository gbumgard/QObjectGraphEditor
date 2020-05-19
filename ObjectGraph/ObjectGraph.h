#ifndef OBJECTGRAPH_H
#define OBJECTGRAPH_H

#include <QObject>
#include <QPluginLoader>
#include <QSqlDatabase>
#include <QMetaObject>
#include <QStringList>
#include <QMap>
#include <QList>
#include <QPair>
#include <memory>
#include <map>

#include "ObjectGraphDatabase.h"
#include "ObjectFactory.h"

class ObjectGraph : public QObject
{
  Q_OBJECT

protected:

  explicit ObjectGraph(QObject *parent = nullptr);


public:

  static std::unique_ptr<ObjectGraph> constructObjectGraph(QObject* parent = nullptr);

  bool loadPluginObjects(const QStringList& paths);

  bool loadFromDatabase(const QString& dbFileName,
                        const QString& sqlDriverName = QString("QSQLITE"));

  QList<QString> objectClassNames() const;

  /*
  bool save(const QString& path);
  bool restore(const QString& path);
  */

  //bool addObjectType(const QMetaObject& metaObject);

  bool containsObjectClass(const QString& className) const;
  bool containsObjectClass(const QMetaObject& metaObject) const;

  /*
  bool removeObjectType(const QString& className);
  bool removeObjectType(const QMetaObject& metaObject);
  */

  QMetaObject getMetaObject(const QString& className) const;

  int createNode(const QString& className);

  QObject* getNode(int nodeId);
  QObject* getNode(const QString& objectName);

  bool containsNode(int nodeId);
  bool containsNode(const QString& objectName) const;
  bool containsNode(const QObject* object) const;

  bool removeNode(int nodeId);
  bool removeNode(const QString& objectName);
  bool removeNode(const QObject* object);

#if 0
  bool containsConverterType(const QString& signalSignature, const QString& slotSignature) const;
  QString getConverterType(const QString& signalSignature, const QString& slotSignature) const;

  QList<QString> getObjectsBySlotSignature(const QString& slotSignature, bool connected = false);

  bool canConnect(const QString& signalSignature, const QString& slotSignature) const;

  bool addConnection(const QString& senderObjectName, const QMetaMethod& signalMethod,
                     const QString& receiverObjectName, const QMetaMethod& slotMethod);
  bool addConnection(QObject* senderObject, const QMetaMethod& signalMethod,
                     QObject* receiverObject, const QMetaMethod& slotMethod);
  bool addConnection(const QString& senderObjectName, const QString& signalSignature,
                     const QString& receiverObjectName, const QString& slotSignature);
  bool addConnection(const QString& senderObjectName, int signalIndex,
                     const QString& receiverObjectName, int slotIndex);
  bool addConnection(QObject* senderObject, const QString& signalSignature,
                     QObject* receiverObject, const QString& slotSignature);
  bool addConnection(QObject* senderObject, int signalIndex,
                     QObject* receiverObject, int slotIndex);

  bool removeConnection(const QString& senderObjectName, const QMetaMethod& signalMethod,
                        const QString& receiverObjectName, const QMetaMethod& slotMethod);
  bool removeConnection(const QString& senderObjectName, const QString& signalSignature,
                        const QString& receiverObjectName, const QString& slotSignature);
  bool removeConnection(const QString& senderObjectName, int signalIndex,
                        const QString& receiverObjectName, int slotIndex);

  bool removeSignalConnections(const QString& senderObjectName, const QString& signalSignature);
  bool removeSignalConnections(const QString& senderObjectName, int signalIndex);

  bool removeSignalConnection(const QString& senderObjectName,
                              const QString& signalSignature,
                              const QString receiverObjectName);

  bool removeSignalConnection(QObject* senderObject,
                              int signalIndex,
                              QObject* receiverObject);

  bool removeSlotConnection(const QString& receiverObjectName, const QString& slotSignature);
  bool removeSlotConnection(const QString& receiverObjectName, int slotIndex);
  bool removeSlotConnection(QObject* receiverObject, const QString& slotSignature);
  bool removeSlotConnection(QObject* receiverObject, int slotIndex);

  bool isSignalConnected(const QString& senderObjectName, const QString& signalSignature) const;
  bool isSignalConnected(const QString* senderObjectName, int signalIndex) const;

  bool isSlotConnected(const QString& receiverObjectName, const QString& slotSignature) const;
  bool isSlotConnected(const QString& receiverObjectName, int slotIndex) const;

  QList<QPair<QString,QString>> signalConnectionsBySignature(const QString& senderObjectName, const QString& signalSignature) const;
  QList<QPair<QObject*,QString>> signalConnectionsBySignature(const QObject* senderObject, const QString& signalSignature) const;
  QList<QPair<QString,int>> signalConnectionsByIndex(const QString& senderObjectName, int signalIndex) const;
  QList<QPair<QObject*,int>> signalConnectionsByIndex(const QObject* senderObject, int signalIndex) const;

  QPair<QString,QString> slotConnectionBySignature(const QString& receiverObjectName, const QString& slotSignature) const;
  QPair<QObject*,QString> slotConnectionBySignature(const QObject* receiverObject, const QString& slotSignature) const;
  QPair<QString,int> slotConnectionsByIndex(const QString& receiverObjectName, int slotIndex) const;
  QPair<QObject*,int> slotConnectionsByIndex(const QObject* receiverObject, int slotIndex) const;

signals:

  void connectionAdded(QObject* sender, const QString& signalSignature,
                       QObject* receiver, const QString& slotSignature);

  void connectionAdded(QObject* sender, int signalIndex,
                       QObject* receiver, int slotIndex);

  void connectionRemoved(QObject* sender, const QString& signalSignature,
                         QObject* receiver, const QString& slotSignature);

  void connectionRemoved(QObject* sender, int signalIndex,
                         QObject* receiver, int slotIndex);

  void objectTypeAdded(QObject* object);
  void objectTypeRemoved(QObject* object);

  void objectAdded(QObject* object);
  void objectRemoved(QObject* object);

public slots:

protected:

private:
#endif

  int _objectIndex;

  std::map<int,std::unique_ptr<QObject>> _objectInstances;

  /*
  static ObjectFactory _objectFactory;

  QMap<QString,ObjectFactory*> _objectFactoryPlugins;

  QMap<QString,ObjectFactory*> _objectTypeFactories;

  QMap<QString,QMetaObject*> _objectTypes;

  //QMap<QString,QMetaObject*> _converterObjectTypes;


  //QMap<QObject*,QMap<int,QList<QPair<QObject*,int>> _signalToSlotMap;

  //QMap<QObject*,QMap<int,QPair<QObject*,int>>> _slotToSignalMap;

  */

  ObjectGraphDatabase _database;

};

#endif // OBJECTGRAPHDATABASE_H
