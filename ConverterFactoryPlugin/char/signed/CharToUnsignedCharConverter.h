#ifndef CHARTOUNSIGNEDCHARCONVERTER_H
#define CHARTOUNSIGNEDCHARCONVERTER_H

#include <QObject>

class CharToUnsignedCharConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(unsigned char)","out")
  Q_CLASSINFO("class-alias","Char to Unsigned Char")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToUnsignedCharConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned char value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOUNSIGNEDCHARCONVERTER_H
