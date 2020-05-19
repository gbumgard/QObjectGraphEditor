#ifndef THREADEDOBJECT_H
#define THREADEDOBJECT_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>

class ThreadedObject : public QThread
{
public:

  ThreadedObject(const ThreadedObject&) = delete;
  ThreadedObject& operator=(const ThreadedObject&) = delete;

  virtual ~ThreadedObject();

public slots:

  void stop();

protected:

  ThreadedObject(QObject* parent = nullptr);

  void run() override;

  virtual void update();

  class UpdateLock {
  public:

    UpdateLock(ThreadedObject* obj)
      : _obj(obj)
    {
      _obj->_mutex.lock();
    }

    ~UpdateLock() {
      _obj->_nextSeqNumber++;
      _obj->_waitCondition.wakeAll();
      _obj->_mutex.unlock();
    }

  private:
    ThreadedObject* _obj;
  };

  class WaitForUpdateLock {
  public:

    WaitForUpdateLock(ThreadedObject* obj)
      : _obj(obj)
    {
      _obj->_mutex.lock();
      if (_obj->_lastSeqNumber == _obj->_nextSeqNumber) {
        _obj->_waitCondition.wait(&_obj->_mutex);
        if (!_obj->_stop) {
          _obj->_lastSeqNumber = _obj->_nextSeqNumber;
        }
      }
    }

    ~WaitForUpdateLock() {
      _obj->_mutex.unlock();
    }

  private:
    ThreadedObject* _obj;
  };

protected:

  bool _stop;
  QMutex _mutex;
  QWaitCondition _waitCondition;
  qint32 _nextSeqNumber;
  qint32 _lastSeqNumber;

};

#endif // THREADEDOBJECT_H
