#include "SignalMimeData.h"

const QString SignalMimeData::MIME_TYPE("application/x-signal-method-index");

SignalMimeData::SignalMimeData(const QUuid& objectUuid, const QString& methodSignature, ObjectGraphEdge* connectionPath)
  : MethodMimeData(objectUuid, methodSignature, connectionPath)
{
}

QStringList SignalMimeData::formats() const {
  QStringList formatList;
  formatList << MIME_TYPE;
  return formatList;
}

bool SignalMimeData::hasFormat(const QString &mimeType) const {
  return mimeType == MIME_TYPE;
}
