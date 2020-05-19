#ifndef INTTOLONGLONGCONVERTER_H
#define INTTOLONGLONGCONVERTER_H

#include <QObject>

class IntToLongLongConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(long long)","out")
  Q_CLASSINFO("class-alias","Int to Long Long")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToLongLongConverter(QObject *parent = nullptr);

signals:

  void conversion(long long value);

public slots:

  void convert(int value) {
    emit conversion(value);
  }
};


#endif // INTTOLONGLONGCONVERTER_H
