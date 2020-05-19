#ifndef INTTOUNSIGNEDINTCONVERTER_H
#define INTTOUNSIGNEDINTCONVERTER_H

#include <QObject>

class IntToUnsignedIntConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(unsigned int)","out")
  Q_CLASSINFO("class-alias","Int to Unsigned Int")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToUnsignedIntConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned int value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};

#endif // INTTOUNSIGNEDINTCONVERTER_H
