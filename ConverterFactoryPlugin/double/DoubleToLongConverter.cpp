#include "DoubleToLongConverter.h"
#include "ConverterFactoryPlugin.h"

REGISTER_CLASS(DoubleToLongConverter)

DoubleToLongConverter::DoubleToLongConverter(QObject *parent)
  : QObject(parent)
{
}
