#ifndef INTTOCHARCONVERTER_H
#define INTTOCHARCONVERTER_H

#include <QObject>

class IntToCharConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(char)","out")
  Q_CLASSINFO("class-alias","Int to Char")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToCharConverter(QObject *parent = nullptr);

signals:

  void conversion(char value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }

};

#endif // INTTOCHARCONVERTER_H
