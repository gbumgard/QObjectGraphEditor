#ifndef CHARTOLONGLONGCONVERTER_H
#define CHARTOLONGLONGCONVERTER_H

#include <QObject>

class CharToLongLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(char)","in")
  Q_CLASSINFO("conversion(long long)","out")
  Q_CLASSINFO("class-alias","Char to Long Long")
  Q_CLASSINFO("directory","Converters/Primitive/Char to ...")

public:

  Q_INVOKABLE explicit CharToLongLongConverter(QObject *parent = nullptr);

signals:

  void conversion(long long value);

public slots:

  void convert(char value) {
    emit conversion(value);
  }

};

#endif // CHARTOLONGLONGCONVERTER_H
