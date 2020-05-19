#ifndef CHARTODOUBLECONVERTER_H
#define CHARTODOUBLECONVERTER_H

#include <QObject>

class CharToDoubleConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(double)","out")
  Q_CLASSINFO("class-alias","Char to Double")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToDoubleConverter(QObject *parent = nullptr);

signals:

  void conversion(double value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTODOUBLECONVERTER_H
