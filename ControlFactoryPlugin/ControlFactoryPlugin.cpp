#include "ControlFactoryPlugin.h"


#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(ControlFactoryPlugin, ControlFactoryPlugin)
#endif // QT_VERSION < 0x050000


ControlFactoryPlugin::ControlFactoryPlugin(QObject *parent)
    : ObjectFactory(parent)
{

}

ControlFactoryPlugin::~ControlFactoryPlugin()
{

}
