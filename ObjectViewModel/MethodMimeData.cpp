#include "MethodMimeData.h"

#include <QDebug>

MethodMimeData::MethodMimeData(int objectId, int methodIndex, ObjectGraphEdge* connectionPath)
  : QMimeData()
  , _objectId(objectId)
  , _methodIndex(methodIndex)
  , _edge(connectionPath)
{
}

MethodMimeData::~MethodMimeData() {
  qDebug() << Q_FUNC_INFO;
}
