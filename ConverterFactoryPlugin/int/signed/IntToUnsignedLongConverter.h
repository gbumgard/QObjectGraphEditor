#ifndef INTTOUNSIGNEDLONGCONVERTER_H
#define INTTOUNSIGNEDLONGCONVERTER_H

#include <QObject>

class IntToUnsignedLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(unsigned long)","out")
  Q_CLASSINFO("class-alias","Int to Unsigned Long")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToUnsignedLongConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned long value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};

#endif // INTTOUNSIGNEDLONGCONVERTER_H
