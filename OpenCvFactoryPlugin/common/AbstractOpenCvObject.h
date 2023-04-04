#ifndef ABSTRACTOPENCVOBJECT_H
#define ABSTRACTOPENCVOBJECT_H

#include <QObject>
#include "OpenCvFactoryPlugin.h"

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

  void updateObject() {
    // TODO: Catch exceptions
    doUpdate();
  }

private:


};


#endif // ABSTRACTOPENCVOBJECT_H
