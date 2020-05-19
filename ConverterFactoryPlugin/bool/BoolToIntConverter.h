#ifndef BOOLTOINTCONVERTER_H
#define BOOLTOINTCONVERTER_H

#include <QObject>

class BoolToIntConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(bool)","in")
  Q_CLASSINFO("conversion(int)","out")
  Q_CLASSINFO("class-alias","Bool to Int")
  Q_CLASSINFO("directory","Converters/Primitive/Bool to ...")

public:

  Q_INVOKABLE explicit BoolToIntConverter(QObject *parent = nullptr);

signals:

  void conversion(int value);

public slots:

  void convert(bool value) {
    emit conversion(value ? 1 : 0);
  }

};

#endif // BOOLTOINTCONVERTER_H
