#ifndef OPENCVFACTORYPLUGIN_H
#define OPENCVFACTORYPLUGIN_H

#include "ObjectFactory.h"
#include "MatEvent.h"
#include "ScalarEvent.h"

#include <QMetaObject>
#include <QMap>
#include <QString>
#include <QDateTime>

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef qint64 TimeStamp;

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(cv::Scalar)
Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGB>)

class OpenCvFactoryPlugin : public ObjectFactory
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID ObjectFactoryPlugin_iid FILE "OpenCvFactoryPlugin.json")

public:

  Q_INVOKABLE OpenCvFactoryPlugin(QObject *parent = 0) : ObjectFactory(parent)
  {
    qDebug() << Q_FUNC_INFO;
    qRegisterMetaType<cv::Mat>();
    qRegisterMetaType<cv::Scalar>();
    qRegisterMetaType<MatEvent>();
    qRegisterMetaType<ScalarEvent>();
    qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGB>>();
  }

  virtual ~OpenCvFactoryPlugin() {}

};


#endif // OPENCVFACTORYPLUGIN_H
