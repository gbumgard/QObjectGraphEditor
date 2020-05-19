#ifndef GRAPHSTYLEPLUGIN_H
#define GRAPHSTYLEPLUGIN_H

#include <QObject>
#include <QStylePlugin>

class GraphStylePlugin : public QStylePlugin
{
  Q_OBJECT

#if QT_VERSION >= 0x050000
  Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QStyleFactoryInterface" FILE "GraphStylePlugin.json")
#endif // QT_VERSION >= 0x050000

public:

  GraphStylePlugin(QObject *parent = 0);

  QStyle *create(const QString &key) override;

};

#endif // GRAPHSTYLEPLUGIN_H
