#include "common/ThreadedObject.h"

ThreadedObject::ThreadedObject(QObject* parent)
  : QThread(parent)
  , _stop(false)
  , _mutex()
  , _waitCondition()
  , _nextSeqNumber(0)
  , _lastSeqNumber(0)
{
}

ThreadedObject::~ThreadedObject() {
  stop();
  wait();
}

void ThreadedObject::run() {
  while (!_stop) {
    update();
  }
}

void ThreadedObject::stop() {
  _stop = true;
  _waitCondition.wakeAll();
}

void ThreadedObject::update() {
  // Declare temp variables
  {
    WaitForUpdateLock lock(this);
    if (_stop) return;
  }
  // Perform computation using temp variables
}
