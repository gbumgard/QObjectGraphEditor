#ifndef CONNECTION_H
#define CONNECTION_H

#include "objectmodel_global.h"
#include <QUuid>
#include <QHash>

/**
 * @brief The Connection class
 * A Connection object describes a connection between a sender/signal and reciever/slot.
 */
class OBJECTMODELSHARED_EXPORT Connection
{
public:

  Connection();

  Connection(const Connection& other);

  Connection(const QUuid& senderUuid,
             const QString& signalSignature,
             const QUuid& receiverUuid,
             const QString& slotSignature,
             const QUuid& connectionUuid);

  const QUuid& senderUuid() const { return _senderUuid; }
  const QString& signalSignature() const { return _signalSignature; }
  const QUuid& receiverUuid() const { return _receiverUuid; }
  const QString& slotSignature() const { return _slotSignature; }
  const QUuid& connectionUuid() const { return _connectionUuid; }

  Connection& operator=(const Connection& other);

  bool operator==(const Connection& other) const;

  private:

  QUuid _senderUuid;
  QString _signalSignature;
  QUuid _receiverUuid;
  QString _slotSignature;
  QUuid _connectionUuid;

};

inline uint qHash(const Connection& connection, int seed) {
  return seed ^
         qHash(connection.senderUuid(),seed) ^
         qHash(connection.signalSignature()) ^
         qHash(connection.receiverUuid(),seed) ^
         qHash(connection.slotSignature(),seed);
}

#endif // CONNECTION_H
