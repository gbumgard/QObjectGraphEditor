#ifndef INTTOQVARIANTCONVERTER_H
#define INTTOQVARIANTCONVERTER_H

#include <QObject>
#include <QVariant>

class IntToQVariantConverter : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("convert(int)","in")
  Q_CLASSINFO("conversion(QVariant)","out")
  Q_CLASSINFO("class-alias","Int To QVariant")
  Q_CLASSINFO("directory","Converters/Primitive/Int to...")

public:

  Q_INVOKABLE explicit IntToQVariantConverter(QObject *parent = nullptr);

signals:

  void conversion(const QVariant& value);

public slots:

  void convert(int value) {
    emit conversion(QVariant(value));
  }

};
;

#endif // INTTOQVARIANTCONVERTER_H
