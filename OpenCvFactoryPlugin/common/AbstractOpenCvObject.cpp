#include "AbstractOpenCvObject.h"
#include <QDynamicPropertyChangeEvent>

AbstractOpenCvObject::AbstractOpenCvObject(QObject *parent)
  : QObject(parent)
{
  setProperty("methodOffset",AbstractOpenCvObject::staticMetaObject.methodOffset());
}

