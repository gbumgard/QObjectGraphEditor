#ifndef CHARTOUNSIGNEDLONGCONVERTER_H
#define CHARTOUNSIGNEDLONGCONVERTER_H

#include <QObject>

class CharToUnsignedLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(unsigned long)","out")
  Q_CLASSINFO("class-alias","Char to Unsigned Long")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToUnsignedLongConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned long value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOUNSIGNEDLONGCONVERTER_H
