#include "Connection.h"

Connection::Connection()
  : _senderUuid()
  , _signalSignature()
  , _receiverUuid()
  , _slotSignature()
  , _connectionUuid()
{

}

Connection::Connection(const Connection& other)
  : _senderUuid(other._senderUuid)
  , _signalSignature(other._signalSignature)
  , _receiverUuid(other._receiverUuid)
  , _slotSignature(other._slotSignature)
  , _connectionUuid(other._connectionUuid)
{

}

Connection::Connection(const QUuid& senderId,
                       const QString& signalName,
                       const QUuid& receiverId,
                       const QString& slotName,
                       const QUuid& connectionId)
  : _senderUuid(senderId)
  , _signalSignature(signalName)
  , _receiverUuid(receiverId)
  , _slotSignature(slotName)
  , _connectionUuid(connectionId)
{

}

Connection& Connection::operator=(const Connection& other) {
  _senderUuid = other._senderUuid;
  _signalSignature = other._slotSignature;
  _receiverUuid = other._receiverUuid;
  _slotSignature = other._slotSignature;
  _connectionUuid = other._connectionUuid;
  return *this;
}

bool Connection::operator==(const Connection& other) const {
  return  _senderUuid == other._senderUuid &&
          _signalSignature == other._slotSignature &&
          _receiverUuid == other._receiverUuid &&
          _slotSignature == other._slotSignature &&
          _connectionUuid == other._connectionUuid;
}
