#ifndef INTTOLONGDOUBLECONVERTER_H
#define INTTOLONGDOUBLECONVERTER_H

#include <QObject>

class IntToLongDoubleConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(long double)","out")
  Q_CLASSINFO("class-alias","Int to Long Double")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToLongDoubleConverter(QObject *parent = nullptr);

signals:

  void conversion(long double value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};


#endif // INTTOLONGDOUBLECONVERTER_H
