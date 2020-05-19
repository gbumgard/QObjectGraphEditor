#ifndef CHARTOLONGCONVERTER_H
#define CHARTOLONGCONVERTER_H

#include <QObject>

class CharToLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(long)","out")
  Q_CLASSINFO("class-alias","Char to Long")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToLongConverter(QObject *parent = nullptr);

signals:

  void conversion(long value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOLONGCONVERTER_H
