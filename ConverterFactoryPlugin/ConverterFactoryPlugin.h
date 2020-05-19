#ifndef CONVERTERFACTORYPLUGIN_H
#define CONVERTERFACTORYPLUGIN_H

#include <QMetaObject>
#include <QMap>
#include <QString>
#include "ObjectFactory.h"

class ConverterFactoryPlugin : public ObjectFactory
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID ObjectFactoryPlugin_iid FILE "ConverterFactoryPlugin.json")

public:

  Q_INVOKABLE ConverterFactoryPlugin(QObject *parent = 0) : ObjectFactory(parent) {}

  virtual ~ConverterFactoryPlugin() {}

};

#endif // CONVERTERFACTORYPLUGIN_H
