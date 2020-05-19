#ifndef CONVERTEROBJECT_H
#define CONVERTEROBJECT_H

#include <QObject>

class ConverterObject : public QObject {

  Q_OBJECT

public:

  ConverterObject(QObject* parent) : QObject(parent) {}

  virtual ~ConverterObject() {}

};

#endif // CONVERTEROBJECT_H
