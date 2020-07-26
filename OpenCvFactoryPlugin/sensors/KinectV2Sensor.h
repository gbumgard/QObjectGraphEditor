#ifndef KINECTV2SENSOR_H
#define KINECTV2SENSOR_H

#include <QObject>

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QMetaType>

#include "OpenCvFactoryPlugin.h"
#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#ifdef WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)
#endif

#include <Eigen/Core>

#include <cstdlib>
#include <string>
#include <iostream>


class KinectV2Sensor: public QThread
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Kinect V2")
  Q_CLASSINFO("directory","OpenCV/Sensors")

  Q_PROPERTY(QString serial READ serial)
  Q_PROPERTY(bool filterUnmatchedPixels READ filterUnmatchedPixels WRITE filterUnmatchedPixels)
  Q_PROPERTY(bool generateHighDef READ generateHighDef WRITE generateHighDef)
  Q_PROPERTY(bool mirror READ mirror WRITE mirror)

public:

  enum Processor{
    CPU, OPENCL, OPENGL, CUDA
  };

  Q_INVOKABLE explicit KinectV2Sensor(QObject* parent = nullptr);

  virtual ~KinectV2Sensor();

  void deviceIndex(int index);

  bool filterUnmatchedPixels() const { return _filterUnmatchedPixels; }
  void filterUnmatchedPixels(bool filterUnmatchedPixels) { _filterUnmatchedPixels = filterUnmatchedPixels; }

  bool generateHighDef() const { return _generateHighDef; }
  void generateHighDef(bool generateHighDef) { _generateHighDef = generateHighDef; }

  bool mirror() const { return _mirror; }
  void mirror(bool mirror) { _mirror = mirror; }

  QString serial() const { return QString(_serial.c_str()); }

public slots:

signals:

#if 0
  void depth(const MatEvent& mat);
  void color(const MatEvent& mat);
  void ir(const MatEvent& mat);
  void undistortedDepth(const MatEvent& mat);
  void registeredColor(const MatEvent& mat);
#endif

  QVARIANT_PAYLOAD(MatEvent) void depth(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void color(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void ir(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void undistortedDepth(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void registeredColor(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void mat(const QVariant& variant);


#ifdef WITH_PCL
  void pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
#endif

protected:

  bool open(const std::string& serial = std::string());

  void close();

  void stop();

  void shutDown();

  void run() override;

  void connectNotify(const QMetaMethod &signal) override;
  void disconnectNotify(const QMetaMethod &signal) override;

  void updateConnectionState();

  void update();

  libfreenect2::Freenect2Device::IrCameraParams getIrParameters();
  libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();

  void disableLog();
  void enableLog();

  void printParameters();
  void storeParameters();

  libfreenect2::SyncMultiFrameListener * getListener();

private:

  void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);

private:

  QMutex _mutex;
  QWaitCondition _waitCondition;
  bool _stop;

  cv::Mat mDepthMap;
  cv::Mat mIrCameraImage;
  cv::Mat mBGRCameraImage;

#ifdef WITH_PCL
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPointCloudMap;
#endif

  bool _updateDepthMapRequired;
  bool _updateIrCameraImageRequired;
  bool _updateBGRCameraImageRequired;
  bool mUpdateUndistortedDepthRequired;
  bool mUpdateRegisteredColorRequired;
  bool _updatePointCloudMapRequired;

  libfreenect2::Freenect2 _freenect2;
  libfreenect2::Freenect2Device * _dev = 0;
  libfreenect2::PacketPipeline * _pipeline = 0;
  libfreenect2::Registration * _registration = 0;
  libfreenect2::SyncMultiFrameListener _listener;
  libfreenect2::Logger * _logger = nullptr;
  libfreenect2::FrameMap _frames;
  libfreenect2::Frame _undistortedDepthFrame;
  libfreenect2::Frame _registeredColorFrame;
  libfreenect2::Frame _bigMat;

  Eigen::Matrix<float,512,1> _colmap;
  Eigen::Matrix<float,424,1> _rowmap;

  std::string _serial;

  int _map[512 * 424];

  float _qnan;

  bool _filterUnmatchedPixels;
  bool _generateHighDef;
  bool _mirror;

};

#endif // KINECTV2SENSOR_H
