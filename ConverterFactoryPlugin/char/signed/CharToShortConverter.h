#ifndef CHARTOSHORTCONVERTER_H
#define CHARTOSHORTCONVERTER_H

#include <QObject>

class CharToShortConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(short)","out")
  Q_CLASSINFO("class-alias","Char to Short")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToShortConverter(QObject* parent = nullptr);

signals:

  void conversion(short value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOSHORTCONVERTER_H
