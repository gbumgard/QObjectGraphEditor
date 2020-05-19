#ifndef INTTOBOOLCONVERTER_H
#define INTTOBOOLCONVERTER_H

#include <QObject>

class IntToBoolConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(bool)","out")
  Q_CLASSINFO("class-alias","Int to Bool")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToBoolConverter(QObject *parent = nullptr);

signals:

  void conversion(bool value);

public slots:

  void convert(int value) {
    emit conversion(value != 0);
  }

};

#endif // INTTOBOOLCONVERTER_H
