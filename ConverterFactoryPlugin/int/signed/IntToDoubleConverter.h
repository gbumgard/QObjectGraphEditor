#ifndef INTTODOUBLECONVERTER_H
#define INTTODOUBLECONVERTER_H

#include <QObject>

class IntToDoubleConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(double)","out")
  Q_CLASSINFO("class-alias","Int to Double")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToDoubleConverter(QObject *parent = nullptr);

signals:

  void conversion(double value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};

#endif // INTTODOUBLECONVERTER_H
