#ifndef DOUBLETOLONGCONVERTER_H
#define DOUBLETOLONGCONVERTER_H

#include <QObject>

class DoubleToLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(double)","double")
  Q_CLASSINFO("conversion(long)","long")
  Q_CLASSINFO("class-alias","Double To Long")
  Q_CLASSINFO("directory","Converters/Primitive/Double to...")

public:

  Q_INVOKABLE explicit DoubleToLongConverter(QObject *parent = nullptr);

signals:

  void conversion(long value);

public slots:

  void convert(double value) {
    emit conversion(value);
  }

};
#endif // DOUBLETOLONGCONVERTER_H
