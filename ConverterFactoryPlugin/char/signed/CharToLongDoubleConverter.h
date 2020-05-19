#ifndef CHARTOLONGDOUBLECONVERTER_H
#define CHARTOLONGDOUBLECONVERTER_H

#include <QObject>

class CharToLongDoubleConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(long double)","out")
  Q_CLASSINFO("class-alias","Char to Long Double")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToLongDoubleConverter(QObject *parent = nullptr);

signals:

  void conversion(long double value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOLONGDOUBLECONVERTER_H
