#include "SlotMimeData.h"

const QString SlotMimeData::MIME_TYPE("application/x-slot-method-index");

SlotMimeData::SlotMimeData(const QUuid& objectUuid, const QString& methodSignature, ObjectGraphEdge* connectionPath)
  : MethodMimeData(objectUuid, methodSignature, connectionPath)
{
}

QStringList SlotMimeData::formats() const {
  QStringList formatList;
  formatList << MIME_TYPE;
  return formatList;
}

bool SlotMimeData::hasFormat(const QString &mimeType) const {
  return mimeType == MIME_TYPE;
}
