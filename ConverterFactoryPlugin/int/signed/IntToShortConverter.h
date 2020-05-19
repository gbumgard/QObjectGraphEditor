#ifndef INTTOSHORTCONVERTER_H
#define INTTOSHORTCONVERTER_H

#include <QObject>

class IntToShortConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(short)","out")
  Q_CLASSINFO("class-alias","Int to Short")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToShortConverter(QObject *parent = nullptr);

signals:

  void conversion(short value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};


#endif // INTTOSHORTCONVERTER_H
