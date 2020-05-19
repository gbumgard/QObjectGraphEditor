#ifndef CONTROLFACTORYPLUGIN_H
#define CONTROLFACTORYPLUGIN_H

#include <QMetaObject>
#include <QMap>
#include <QString>
#include "ObjectFactory.h"

class ControlFactoryPlugin : public ObjectFactory
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID ObjectFactoryPlugin_iid FILE "ControlFactoryPlugin.json")

public:

  Q_INVOKABLE ControlFactoryPlugin(QObject *parent = 0) : ObjectFactory(parent) {}

  virtual ~ControlFactoryPlugin() {}

};

#endif // CONTROLFACTORYPLUGIN_H
