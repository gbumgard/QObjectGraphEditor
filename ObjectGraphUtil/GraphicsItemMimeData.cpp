#include "GraphicsItemMimeData.h"

GraphicsItemMimeData::GraphicsItemMimeData(QGraphicsItem* widget,
                                           const QString& mimeType)
  : QMimeData()
  , _item(widget)
  , _mimeType(mimeType)
{
}

QStringList GraphicsItemMimeData::formats() const {
  QStringList list;
  list << _mimeType;
  return list;
}

bool GraphicsItemMimeData::hasFormat(const QString& mimeType) const {
  return _mimeType == mimeType;
}
