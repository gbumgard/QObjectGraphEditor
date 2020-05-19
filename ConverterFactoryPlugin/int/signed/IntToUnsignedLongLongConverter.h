#ifndef INTTOUNSIGNEDLONGLONGCONVERTER_H
#define INTTOUNSIGNEDLONGLONGCONVERTER_H

#include <QObject>

class IntToUnsignedLongLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(unsigned long long)","out")
  Q_CLASSINFO("class-alias","Int to Unsigned Long Long")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToUnsignedLongLongConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned long long value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};

#endif // INTTOUNSIGNEDLONGLONGCONVERTER_H
