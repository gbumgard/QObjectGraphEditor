#ifndef DOUBLETOINTCONVERTER_H
#define DOUBLETOINTCONVERTER_H

#include <QObject>

class DoubleToIntConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(double)","double")
  Q_CLASSINFO("conversion(int)","int")
  Q_CLASSINFO("class-alias","Double To Int")
  Q_CLASSINFO("directory","Converters/Primitive/Double to...")

public:

  Q_INVOKABLE explicit DoubleToIntConverter(QObject *parent = nullptr);

signals:

  void conversion(int value);

public slots:

  void convert(double value) {
    emit conversion(value);
  }

};

#endif // DOUBLETOINTCONVERTER_H
