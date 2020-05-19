#ifndef OBJECTFACTORY_H
#define OBJECTFACTORY_H

#include "objectmodel_global.h"

#include <QObject>
#include <QPluginLoader>
#include <QString>
#include <QList>
#include <QDir>
#include <QMetaClassInfo>
#include <QDebug>
#include <iostream>

#define ObjectFactoryPlugin_iid "com.larkwoodlabs.qt.plugins.ObjectFactory"

class OBJECTMODELSHARED_EXPORT ObjectFactory : public QObject
{
  Q_OBJECT

public:

  ObjectFactory(QObject *parent = 0) : QObject(parent) {}

  virtual ~ObjectFactory() {}

  static QObject* create(const QString& name) {
    qDebug() << Q_FUNC_INFO;
    if (metaObjectMap().contains(name)) {
      QMetaObject& metaObject = metaObjectMap()[name];
      QObject* qobject = metaObject.newInstance();
      if (!qobject) {
        std::cerr << Q_FUNC_INFO
                  << " could not create "
                  << name.toLatin1().constData()
                  << " instance - is the constructor method tagged with Q_INVOKABLE?"
                  << std::endl;
        return nullptr;
      }
      return qobject;
    }
    return nullptr;
  }

  static QList<QString> names() {
    return metaObjectMap().keys();
  }

  static bool addObjectType(const QMetaObject& metaObject) {
    qDebug() << Q_FUNC_INFO << metaObject.className();
    metaObjectMap().insert(metaObject.className(),metaObject);
    return true;
  }

  static bool removeObjectType(const QString& className) {
    return metaObjectMap().remove(className) == 1;
  }

  static QMetaObject getObjectType(const QString& className) {
    if (metaObjectMap().contains(className)) {
      return metaObjectMap().value(className);
    }
    return QMetaObject();
  }

  static bool contains(const QString& className) {
    return metaObjectMap().contains(className);
  }

  static int loadObjectFactoryPlugins(const QStringList& pluginDirList) {
    qDebug() << Q_FUNC_INFO << pluginDirList;
    int count = 0;
    for (auto path : pluginDirList) {
      QDir pluginDir(path);
      foreach (QString fileName, pluginDir.entryList(QDir::Files)) {
        QString path = pluginDir.absoluteFilePath(fileName);
        qDebug() << Q_FUNC_INFO << path;
        QPluginLoader pluginLoader(path);
        pluginLoader.load();
        bool isLoaded = pluginLoader.isLoaded();
        if (!isLoaded) {
          qDebug() << "could not load" << fileName << "as plugin library";
          continue;
        }
        QObject* plugin = pluginLoader.instance();
        if (!plugin) {
          qDebug() << "could not create" << fileName << "plugin";
          continue;
        }
        ObjectFactory* factory = dynamic_cast<ObjectFactory*>(plugin);
        if (!factory) {
          qDebug() << "plugin library" << fileName << "does not contain an ObjectFactory plugin";
          continue;
        }
        count++;
      }
    }
    return count;
  }

protected:

  static QMap<QString,QMetaObject>& metaObjectMap() {
    static QMap<QString,QMetaObject> map;
    return map;
  }

};

#define REGISTER_CLASS(qobject) static bool init = ObjectFactory::addObjectType(qobject::staticMetaObject);

#endif // OBJECTFACTORYPLUGIN_H
