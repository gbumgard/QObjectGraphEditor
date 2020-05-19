#ifndef INTTOUNSIGNEDCHARCONVERTER_H
#define INTTOUNSIGNEDCHARCONVERTER_H

#include <QObject>

class IntToUnsignedCharConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(unsigned char)","out")
  Q_CLASSINFO("class-alias","Int to Unsigned Char")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToUnsignedCharConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned char value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};


#endif // INTTOUNSIGNEDCHARCONVERTER_H
