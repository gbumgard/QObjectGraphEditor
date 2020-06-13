#ifndef OPENCVFACTORYPLUGIN_H
#define OPENCVFACTORYPLUGIN_H

#include <QMetaObject>
#include <QMap>
#include <QString>
#include "ObjectFactory.h"
#include <opencv2/core.hpp>

typedef std::pair<cv::Mat,int> TaggedMat;

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(TaggedMat)

class OpenCvFactoryPlugin : public ObjectFactory
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID ObjectFactoryPlugin_iid FILE "OpenCvFactoryPlugin.json")

public:

  Q_INVOKABLE OpenCvFactoryPlugin(QObject *parent = 0) : ObjectFactory(parent)
  {
    qRegisterMetaType<cv::Mat>();
    qRegisterMetaType<TaggedMat>();
  }

  virtual ~OpenCvFactoryPlugin() {}

  static int nextTag();

};


#endif // OPENCVFACTORYPLUGIN_H
