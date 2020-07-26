#include "MethodMimeData.h"

#include <QDebug>

MethodMimeData::MethodMimeData(const QUuid& objectUuid, const QString& methodSignature, ObjectGraphEdge* connectionPath)
  : QMimeData()
  , _objectUuid(objectUuid)
  , _methodSignature(methodSignature)
  , _edge(connectionPath)
{
}

MethodMimeData::~MethodMimeData() {
  qDebug() << Q_FUNC_INFO;
}
