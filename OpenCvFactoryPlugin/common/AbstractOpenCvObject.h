#ifndef ABSTRACTOPENCVOBJECT_H
#define ABSTRACTOPENCVOBJECT_H

#include <QObject>

class AbstractOpenCvObject : public QObject
{
  Q_OBJECT

public:

  virtual ~AbstractOpenCvObject() {}

signals:


public slots:


protected:

  explicit AbstractOpenCvObject(QObject *parent = nullptr);

  virtual void update() {}

};

#endif // ABSTRACTOPENCVOBJECT_H
