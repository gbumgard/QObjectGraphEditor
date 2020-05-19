#ifndef CHARTOUNSIGNEDLONGLONG_H
#define CHARTOUNSIGNEDLONGLONG_H

#include <QObject>

class CharToUnsignedLongLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(unsigned long long)","out")
  Q_CLASSINFO("class-alias","Char to Unsigned Long Long")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToUnsignedLongLongConverter(QObject *parent = nullptr);

signals:

  void conversion(unsigned long long value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }
};

#endif // CHARTOUNSIGNEDLONGLONG_H
