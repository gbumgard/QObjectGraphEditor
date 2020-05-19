#ifndef CHARTOUNSIGNEDINTCONVERTER_H
#define CHARTOUNSIGNEDINTCONVERTER_H

#include <QObject>

class CharToUnsignedIntConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(unsigned int)","out")
  Q_CLASSINFO("class-alias","Char to Unsigned Int")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToUnsignedIntConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned int value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOUNSIGNEDINTCONVERTER_H
