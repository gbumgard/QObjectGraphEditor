#ifndef KINECTV1SENSOR_H
#define KINECTV1SENSOR_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>

#include <opencv2/core.hpp>
#include <libfreenect/libfreenect.hpp>

#ifdef WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

class FreenectDeviceImpl;

class KinectV1Sensor : public QThread
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Kinect V1")
  Q_CLASSINFO("directory","OpenCV/Sensors")

  Q_PROPERTY(int deviceIndex READ deviceIndex WRITE setDeviceIndex)
  Q_PROPERTY(DepthFormat depthFormat READ depthFormat WRITE depthFormat)
  Q_PROPERTY(ImageFormat imageFormat READ imageFormat WRITE imageFormat)

public:

  enum DepthFormat {
    DEPTH_10_BIT = FREENECT_DEPTH_10BIT,               // = 1, /**< 10 bit depth information in one uint16_t/pixel */
    DEPTH_11_BIT = FREENECT_DEPTH_11BIT,               // = 0, /**< 11 bit depth information in one uint16_t/pixel */
    DEPTH_REGISTERED = FREENECT_DEPTH_REGISTERED,      // = 4, /**< processed depth data in mm, aligned to 640x480 RGB */
    DEPTH_MM = FREENECT_DEPTH_MM                       // = 5, /**< depth to each pixel in mm, but left unaligned to RGB image */
  };

  Q_ENUM(DepthFormat)

  enum ImageFormat {
    RGB = FREENECT_VIDEO_RGB,
    IR_8BIT = FREENECT_VIDEO_IR_8BIT,
    IR_10BIT = FREENECT_VIDEO_IR_10BIT
  };

  Q_ENUM(ImageFormat)

  Q_INVOKABLE explicit KinectV1Sensor(QObject* parent = nullptr);

  virtual ~KinectV1Sensor();

  int deviceIndex() const { return _deviceIndex; }
  void setDeviceIndex(int deviceIndex);

  DepthFormat depthFormat() const { return _depthFormat; }
  void depthFormat(DepthFormat depthFormat);

  ImageFormat imageFormat() const { return _imageFormat; }
  void imageFormat(ImageFormat imageFormat);

signals:

  void depth(const cv::Mat& mat);

  void image(const cv::Mat& mat);

#ifdef WITH_PCL
  void pointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
#endif

protected:

  void run() override;

  void stop();

  void connectNotify(const QMetaMethod &signal) override;
  void disconnectNotify(const QMetaMethod &signal) override;

  void updateConnectionState();

  void update();

private:

  Freenect::Freenect _freenect;
  FreenectDeviceImpl* _device;
  int _deviceIndex;

  QMutex _mutex;
  QWaitCondition _waitCondition;
  bool _stop;

  DepthFormat _depthFormat;
  ImageFormat _imageFormat;

  cv::Mat _depthFrame;
  cv::Mat _colorFrame;

#ifdef WITH_PCL
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointCloudMap;
#endif

  bool mUpdateDepthRequired;
  bool mUpdateImageRequired;
  bool mUpdatePointCloudRequired;
};

#endif // KINECTV1SENSOR_H
