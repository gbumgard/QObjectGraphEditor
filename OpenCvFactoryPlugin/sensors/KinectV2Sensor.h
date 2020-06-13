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

  bool filterUnmatchedPixels() const { return mFilterUnmatchedPixels; }
  void filterUnmatchedPixels(bool filterUnmatchedPixels) { mFilterUnmatchedPixels = filterUnmatchedPixels; }

  bool generateHighDef() const { return mGenerateHighDef; }
  void generateHighDef(bool generateHighDef) { mGenerateHighDef = generateHighDef; }

  bool mirror() const { return mirror_; }
  void mirror(bool mirror) { mirror_ = mirror; }

  QString serial() const { return QString(serial_.c_str()); }

public slots:

signals:

  void depth(const TaggedMat& mat);
  void color(const TaggedMat& mat);
  void ir(const TaggedMat& mat);
  void undistortedDepth(const TaggedMat& mat);
  void registeredColor(const TaggedMat& mat);


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

  QMutex mMutex;
  QWaitCondition mWaitCondition;
  bool mStop;

  cv::Mat mDepthMap;
  cv::Mat mIrCameraImage;
  cv::Mat mBGRCameraImage;

#ifdef WITH_PCL
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPointCloudMap;
#endif

  bool mUpdateDepthMapRequired;
  bool mUpdateIrCameraImageRequired;
  bool mUpdateBGRCameraImageRequired;
  bool mUpdateUndistortedDepthRequired;
  bool mUpdateRegisteredColorRequired;
  bool mUpdatePointCloudMapRequired;

  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device * dev_ = 0;
  libfreenect2::PacketPipeline * pipeline_ = 0;
  libfreenect2::Registration * registration_ = 0;
  libfreenect2::SyncMultiFrameListener listener_;
  libfreenect2::Logger * logger_ = nullptr;
  libfreenect2::FrameMap frames_;
  libfreenect2::Frame mUndistortedDepthFrame;
  libfreenect2::Frame mRegisteredColorFrame;
  libfreenect2::Frame big_mat_;

  Eigen::Matrix<float,512,1> colmap;
  Eigen::Matrix<float,424,1> rowmap;

  std::string serial_;

  int map_[512 * 424];

  float qnan_;

  bool mFilterUnmatchedPixels;
  bool mGenerateHighDef;
  bool mirror_;

};

#endif // KINECTV2SENSOR_H
