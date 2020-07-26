#ifndef OBJECTMODEL_H
#define OBJECTMODEL_H

#include "objectmodel_global.h"
#include "Connection.h"

#include <QMetaType>
#include <QObject>
#include <QMap>
#include <QSet>
#include <QPair>
#include <QVariant>

class Connection;

class OBJECTMODELSHARED_EXPORT ObjectModel
    : public QObject
{

    Q_OBJECT

  public:

    enum Version {
      VERSION_0_1_0 = 0x000100,
      CURRENT_VERSION = VERSION_0_1_0
    };

    enum StatusCode {
      STATUS_UNDEFINED,
      STATUS_OK,
      STATUS_ERROR,
      STATUS_EXCEPTION,
      STATUS_INVALID_SLOT_ARGUMENT_TYPE,
      STATUS_INVALID_SLOT_ARGUMENT_FORMAT
    };

    Q_ENUM(StatusCode)

    ObjectModel(QObject* parent = nullptr);

    virtual ~ObjectModel();

    bool serialize(QDataStream& out,
                   QSet<QUuid>& objectUuidSet,
                   bool allConnections) const;

    bool deserialize(QDataStream& in,
                     bool makeUnique,
                     QMap<QUuid, QUuid>& uuidMap);

    void clear();

    void dump() const;


    /***********************************
     * Methods for managing objects.
     ***********************************/


    void createObject(const QString& objectClassName, const QUuid& objectUuid);

    bool removeObject(const QUuid& objectUuid);

    bool removeObjects(QSet<QUuid>& objectUuids);

    bool containsObject(const QUuid& objectUuid) const;

    bool containsObject(const QObject* const object) const;

    QObject* object(const QUuid &objectUuid) const;

    QUuid objectUuid(const QObject* const object) const;

    QSet<QUuid> objectUuids() const;

    QString objectClassName(const QUuid &objectUuid) const;



    /***********************************
     * Methods for managing connections.
     ***********************************/


    void createConnection(const QUuid& connectionUuid,
                          const QUuid& senderUuid,
                          const QString& signalSignature,
                          const QUuid& receiverUuid,
                          const QString& slotSignature);


    bool removeConnection(const QUuid& connectionUuid);

    bool removeConnections(QSet<QUuid>& connectionUuids);

    bool containsConnection(const QUuid& connectionUuid) const;

    bool containsConnection(const Connection& connection) const;

    Connection connection(const QUuid& connectionUuid) const;

    QSet<Connection> connections(const QSet<QUuid>& connectionUuids) const;

    QSet<QUuid> connectionUuids() const;

    QSet<QUuid> connectionUuids(const QUuid& objectUuid) const;

    QSet<QUuid> connectionUuids(const QUuid& objectUuid, const QString &methodSignature) const;



    /**********************************************
     * Convenience functions for setting predefined
     * dynamic properties on an object.
     **********************************************/


    static QUuid getObjectIdProperty(const QObject* obj);
    static void setObjectIdProperty(QObject* obj, const QUuid& objectUuid);

    static ObjectModel* getObjectModelProperty(const QObject* obj);
    static void setObjectModelProperty(QObject* obj, const ObjectModel* model);

    static QString getObjectCaptionProperty(const QObject* obj);
    static void setObjectCaptionProperty(QObject* obj, const QString& caption);

    static StatusCode getObjectStatusCodeProperty(const QObject* obj);
    static void setObjectStatusCodeProperty(QObject* obj, StatusCode statusCode);

    static QString getObjectStatusMessageProperty(const QObject* obj);
    static void setObjectStatusMessageProperty(QObject* obj, const QString& statusMessage);

    static void setObjectStatus(QObject* obj, StatusCode statusCode, const QString& statusMessage = "OK");

    bool canConnect(const QUuid& senderUuid,
                    const QString& signalSignature,
                    const QUuid& receiverUuid,
                    const QString& slotSignature) const;

  signals:

    /**
     * @brief objectAdded
     * @param objectUuid
     */
    void objectAdded(const QUuid& objectUuid);

    /**
     * @brief objectRemoved
     * @param objectUuid
     */
    void objectRemoved(const QUuid& objectUuid);

    /**
     * @brief connectionAdded
     * @param connectionUuid
     */
    void connectionAdded(const QUuid& connectionUuid);

    /**
     * @brief connectionRemoved
     * @param connectionUuid
     */
    void connectionRemoved(const QUuid& connectionUuid);

    /**
     * @brief objectStatusChanged
     * @param objectUuid
     */
    void objectStatusChanged(const QUuid& objectUuid);


  protected:

    static constexpr const char* OBJECT_ID_PROPERTY_NAME = "x-object-id";
    static constexpr const char* OBJECT_MODEL_PROPERTY_NAME = "x-object-model";
    static constexpr const char* OBJECT_CAPTION_PROPERTY_NAME = "x-object-caption";
    static constexpr const char* STATUS_CODE_PROPERTY_NAME = "x-status-code";
    static constexpr const char* STATUS_MESSAGE_PROPERTY_NAME = "x-status-message";


    bool connect(const QUuid& senderUuid,
                 const QString& signalSignature,
                 const QUuid& receiverUuid,
                 const QString& slotSignature) const;

    bool disconnect(const QUuid& senderUuid,
                    const QString& signalSignature,
                    const QUuid& receiverUuid,
                    const QString& slotSignature) const;


    static void updateObjectStatus(QObject* obj);


  private:

    static QUuid _uuid;

    /**
     * @brief _objects
     * Maps object UUIDs to objects.
     */
    QMap<QUuid,QObject*> _objectIndex;

    /**
     * @brief _connectionIndex
     * Maps connection UUIDs to connections.
     */
    QMap<QUuid,Connection> _connectionIndex;

    /**
     * @brief _objectConnectionIndex
     * Maps object UUIDs to method-signatures and UUIDs for connections..
     */
    QMap<QUuid,QMap<QString,QSet<QUuid>>> _objectConnectionIndex;

};

Q_DECLARE_METATYPE(ObjectModel*)

#endif // OBJECTMODEL_H
