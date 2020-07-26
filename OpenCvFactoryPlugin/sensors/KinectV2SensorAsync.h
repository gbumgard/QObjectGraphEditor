#ifndef KINECTV2SENSORASYNC_H
#define KINECTV2SENSORASYNC_H

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cstdlib>
#include <string>
#include <iostream>


class KinectV2SensorAsync: public QObject, public libfreenect2::SyncMultiFrameListener
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Kinect V2 Async")
  Q_CLASSINFO("directory","OpenCV/Sensors")
  Q_CLASSINFO("signal-order","color(QVariant),depth(QVariant),ir(QVariant),undistorted(QVariant),registered(QVariant),pcl(QVariant)")

  Q_PROPERTY(QString serial READ serial)
  Q_PROPERTY(bool filterUnmatched READ filterUnmatched WRITE filterUnmatched STORED true FINAL)
  Q_PROPERTY(bool generateHighDef READ generateHighDef WRITE generateHighDef STORED true FINAL)
  Q_PROPERTY(bool mirror READ mirror WRITE mirror STORED true FINAL)
  Q_PROPERTY(ColorFormat colorFormat READ colorFormat WRITE colorFormat STORED true FINAL)

public:

  enum Processor{
    CPU, OPENCL, OPENGL, CUDA
  };

  enum ColorFormat {
    COLOR_RGB = libfreenect2::Frame::RGBX,
    COLOR_BGR = libfreenect2::Frame::BGRX
  };

  Q_ENUM(ColorFormat)

  Q_INVOKABLE explicit KinectV2SensorAsync(QObject* parent = nullptr);

  virtual ~KinectV2SensorAsync();

  void deviceIndex(int index);

  bool filterUnmatched() const { return _filterUnmatched; }
  void filterUnmatched(bool filterUnmatched) { _filterUnmatched = filterUnmatched; }

  bool generateHighDef() const { return _generateHighDef; }
  void generateHighDef(bool generateHighDef) { _generateHighDef = generateHighDef; }

  bool mirror() const { return _mirror; }
  void mirror(bool mirror) { _mirror = mirror; }

  ColorFormat colorFormat() const { return _colorFormat; }
  void colorFormat(ColorFormat format) { _colorFormat = format; }

  QString serial() const { return QString(_serialNumber.c_str()); }

public slots:

signals:


  QVARIANT_PAYLOAD(MatEvent) void depth(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void color(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void ir(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void undistorted(const QVariant& mat);
  QVARIANT_PAYLOAD(MatEvent) void registered(const QVariant& mat);

  QVARIANT_PAYLOAD(pcl::PointCloud<pcl::PointXYZRGB>) void pcl(const QVariant& cloud);

protected:

  /**
   * @brief KinectV2SensorAsync::open
   * Opens the first available Kinect2 device.
   * Returns false if there are no free Kinect2 devices or the first available device cannot be opened.
   * @return
   */
  bool open();

  void close();

  virtual bool onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame* frame) override;

  void connectNotify(const QMetaMethod &signal) override;
  void disconnectNotify(const QMetaMethod &signal) override;

  void updateConnectionState();

  libfreenect2::Freenect2Device::IrCameraParams getIrParameters();
  libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();

  void disableLog();
  void enableLog();

  void printParameters();
  void storeParameters();

private:

  void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);

private:

  static std::set<std::string> _openDeviceSerialNumbers;

  pcl::PointCloud<pcl::PointXYZRGB> _pointCloudMap;

  bool _updateDepthMapRequired;
  bool _updateIrCameraImageRequired;
  bool _updateBGRCameraImageRequired;
  bool _updateUndistortedDepthRequired;
  bool _updateRegisteredColorRequired;
  bool _updatePointCloudMapRequired;

  libfreenect2::Freenect2 _freenect2;
  libfreenect2::Freenect2Device* _dev;
  libfreenect2::PacketPipeline* _pipeline;
  libfreenect2::Registration* _registration;

  libfreenect2::Logger * _logger = nullptr;

  libfreenect2::Frame* _colorFrame;
  libfreenect2::Frame* _irFrame;
  libfreenect2::Frame* _depthFrame;
  libfreenect2::Frame _undistortedDepthFrame;
  libfreenect2::Frame _registeredColorFrame;
  libfreenect2::Frame _bigDepthFrame;

  uint32_t _currentSequenceNumber;

  Eigen::Matrix<float,512,1> _colmap;
  Eigen::Matrix<float,424,1> _rowmap;

  std::string _serialNumber;

  int _map[512 * 424];

  float _qnan;

  bool _filterUnmatched;
  bool _generateHighDef;
  bool _mirror;
  ColorFormat _colorFormat;
  QMutex _mutex;
};

#endif // KINECTV2SENSOR_H
