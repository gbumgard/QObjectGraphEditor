#include "IntToQVariantConverter.h"
#include "ConverterFactoryPlugin.h"

REGISTER_CLASS(IntToQVariantConverter)

IntToQVariantConverter::IntToQVariantConverter(QObject *parent)
  : QObject(parent)
{

}
