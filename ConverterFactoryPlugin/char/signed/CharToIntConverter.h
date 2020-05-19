#ifndef CHARTOINTCONVERTER_H
#define CHARTOINTCONVERTER_H

#include <QObject>

class CharToIntConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(int)","out")
  Q_CLASSINFO("class-alias","Char to Int")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToIntConverter(QObject *parent = nullptr);

signals:

  void conversion(int value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOINTCONVERTER_H
