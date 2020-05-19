#ifndef CONNECTION_H
#define CONNECTION_H

#include "objectmodel_global.h"

class OBJECTMODELSHARED_EXPORT Connection
{
public:

  Connection(int senderId = -1,
             int signalIndex = -1,
             int receiverId = -1,
             int slotIndex = -1,
             int connectionId = -1);

  int senderId() const { return _senderId; }
  int signalIndex() const { return _signalIndex; }
  int receiverId() const { return _receiverId; }
  int slotIndex() const { return _slotIndex; }
  int connectionId() const { return _connectionId; }

private:

  int _senderId;
  int _signalIndex;
  int _receiverId;
  int _slotIndex;
  int _connectionId;

};

#endif // CONNECTION_H
