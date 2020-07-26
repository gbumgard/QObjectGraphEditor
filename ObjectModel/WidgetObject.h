#ifndef ABSTRACTOBJECT_H
#define ABSTRACTOBJECT_H

#include <QObject>

class WidgetObject : public QObject
{
  Q_OBJECT

protected:

  explicit WidgetObject(QObject *parent) : QObject(parent)
  {

  }

signals:

};



#endif // ABSTRACTOBJECT_H
