#ifndef INTTOLONGCONVERTER_H
#define INTTOLONGCONVERTER_H

#include <QObject>

class IntToLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(long)","out")
  Q_CLASSINFO("class-alias","Int to Long")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToLongConverter(QObject *parent = nullptr);

signals:

  void conversion(long value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }

};

#endif // INTTOLONGCONVERTER_H
