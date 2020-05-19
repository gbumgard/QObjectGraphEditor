#ifndef METHODMIMEDATA_H
#define METHODMIMEDATA_H

#include <QMimeData>

class ObjectModel;
class ObjectGraphEdge;

class MethodMimeData : public QMimeData
{
public:

  virtual ~MethodMimeData();

  int objectId() const { return _objectId; }
  int methodIndex() const { return _methodIndex; }
  ObjectGraphEdge* edge() const { return _edge; }

protected:

  MethodMimeData(int objectId, int methodIndex, ObjectGraphEdge* edge);

private:

  int _objectId;
  int _methodIndex;
  ObjectGraphEdge* _edge;

};

#endif // METHODMIMEDATA_H
