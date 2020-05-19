#ifndef CHARTOFLOATCONVERTER_H
#define CHARTOFLOATCONVERTER_H

#include <QObject>

class CharToFloatConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(float)","out")
  Q_CLASSINFO("class-alias","Char to Float")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToFloatConverter(QObject *parent = nullptr);

signals:

  void conversion(float value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOFLOATCONVERTER_H
