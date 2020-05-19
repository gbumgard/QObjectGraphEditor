#include "SignalMimeData.h"

const QString SignalMimeData::MIME_TYPE("application/x-signal-method-index");

SignalMimeData::SignalMimeData(int objectId, int methodIndex, ObjectGraphEdge* connectionPath)
  : MethodMimeData(objectId, methodIndex, connectionPath)
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
