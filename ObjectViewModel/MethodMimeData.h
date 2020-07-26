#ifndef METHODMIMEDATA_H
#define METHODMIMEDATA_H

#include <QMimeData>
#include <QUuid>

class ObjectModel;
class ObjectGraphEdge;

class MethodMimeData : public QMimeData
{
public:

  virtual ~MethodMimeData();

  QUuid objectUuid() const { return _objectUuid; }
  QString methodSignature() const { return _methodSignature; }
  ObjectGraphEdge* edge() const { return _edge; }

protected:

  MethodMimeData(const QUuid& objectUuid, const QString& methodSignature, ObjectGraphEdge* edge);

private:

  QUuid _objectUuid;
  QString _methodSignature;
  ObjectGraphEdge* _edge;

};

#endif // METHODMIMEDATA_H
