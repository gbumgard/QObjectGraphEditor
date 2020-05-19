#ifndef SIGNALMIMEDATA_H
#define SIGNALMIMEDATA_H

#include <MethodMimeData.h>

class SignalMimeData : public MethodMimeData
{

public:

  static const QString MIME_TYPE;

  SignalMimeData(int objectId, int methodIndex, ObjectGraphEdge* edge);

  QStringList formats() const override;
  bool hasFormat(const QString& mimeType) const override;

};

#endif // SIGNALMIMEDATA_H
