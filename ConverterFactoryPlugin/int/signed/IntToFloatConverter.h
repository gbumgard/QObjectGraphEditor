#ifndef INTTOFLOATCONVERTER_H
#define INTTOFLOATCONVERTER_H

#include <QObject>

class IntToFloatConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(float)","out")
  Q_CLASSINFO("class-alias","Int to Float")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToFloatConverter(QObject *parent = nullptr);

signals:

  void conversion(float value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }

};

#endif // INTTOFLOATCONVERTER_H
