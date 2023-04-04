#include "AbstractOpenCvObject.h"
#include <QDynamicPropertyChangeEvent>
#include <QVariant>

AbstractOpenCvObject::AbstractOpenCvObject(QObject *parent)
  : QObject(parent)
{
    setProperty("methodOffset",QVariant::fromValue(AbstractOpenCvObject::staticMetaObject.methodOffset()));
}

