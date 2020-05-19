#include "Connection.h"

Connection::Connection(int senderId,
                       int signalIndex,
                       int receiverId,
                       int slotIndex,
                       int connectionId)
  : _senderId(senderId)
  , _signalIndex(signalIndex)
  , _receiverId(receiverId)
  , _slotIndex(slotIndex)
  , _connectionId(connectionId)
{

}
