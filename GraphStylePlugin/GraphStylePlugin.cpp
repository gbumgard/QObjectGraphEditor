#include "GraphStylePlugin.h"


GraphStylePlugin::GraphStylePlugin(QObject *parent) :
  QStylePlugin(parent)
{
}

QStyle *GraphStylePlugin::create(const QString &key)
{
    if (key.toLower() == "node")
        return new MyStyle;
    return 0;
}

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(GraphStylePlugin, GraphStylePlugin)
#endif // QT_VERSION < 0x050000
