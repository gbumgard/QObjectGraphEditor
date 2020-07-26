#ifndef ABSTRACTOPENCVOBJECT_H
#define ABSTRACTOPENCVOBJECT_H

#include "OpenCvFactoryPlugin.h"
#include "MatEvent.h"

#include <QObject>

#include "Event.h"

class AbstractOpenCvObject : public QObject
{
  Q_OBJECT

public:

  virtual ~AbstractOpenCvObject() {}

signals:

public slots:


protected:

  explicit AbstractOpenCvObject(QObject *parent = nullptr);

  virtual void doUpdate() {}

  void update() {
    // TODO: Catch exceptions
    doUpdate();
  }

private:


};


#endif // ABSTRACTOPENCVOBJECT_H
