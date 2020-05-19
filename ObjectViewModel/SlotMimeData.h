#ifndef SLOTMIMEDATA_H
#define SLOTMIMEDATA_H

#include <MethodMimeData.h>

class SlotMimeData : public MethodMimeData
{

public:

  static const QString MIME_TYPE;

  SlotMimeData(int objectId, int methodIndex, ObjectGraphEdge* edge);

  QStringList formats() const override;
  bool hasFormat(const QString& mimeType) const override;

};

#endif // SLOTMIMEDATA_H
