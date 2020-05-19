#ifndef KINECTIMAGESOURCE_H
#define KINECTIMAGESOURCE_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

class OpenNiSensor: public QThread
{

public:

  Q_OBJECT

  Q_CLASSINFO("datasource","true")
  Q_CLASSINFO("class-alias","OpenNI Sensor")
  Q_CLASSINFO("directory","OpenCV/Sensors")

  Q_PROPERTY(int deviceIndex READ deviceIndex WRITE deviceIndex)
  /*
  Q_PROPERTY(cv::Mat currentDepthMap READ currentDepthMap)
  Q_PROPERTY(cv::Mat currentValidDepthMask READ currentValidDepthMask)
  Q_PROPERTY(cv::Mat currentPointCloudMap READ currentPointCloudMap)
  Q_PROPERTY(cv::Mat currentDisparityMap READ currentDisparityMap)
  Q_PROPERTY(cv::Mat currentDisparityMap32F READ currentDisparityMap32F)
  Q_PROPERTY(cv::Mat currentIrCameraImage READ currentIrCameraImage)
  Q_PROPERTY(cv::Mat currentBgrCameraImage READ currentBgrCameraImage)
  Q_PROPERTY(cv::Mat currentGrayCameraImage READ currentGrayCameraImage)
  */

public:

  Q_INVOKABLE explicit OpenNiSensor(QObject* parent = nullptr);

  virtual ~OpenNiSensor();

  bool depthMapsAvailable() const
  {
    return mDepthGeneratorPresent;
  }
  bool validDepthMaskAvailable() const
  {
    return mDepthGeneratorPresent;
  }
  bool pointCloudMapsAvailable() const
  {
    return mDepthGeneratorPresent;
  }
  bool disparityMapsAvailable() const
  {
    return mDepthGeneratorPresent;
  }
  bool irCameraImagesAvailable() const
  {
    return mIrImageGeneratorPresent;
  }
  bool bgrCameraImagesAvailable() const
  {
    return mColorImageGeneratorPresent;
  }
  bool grayCameraImagesAvailable() const
  {
    return mColorImageGeneratorPresent;
  }

  cv::Mat currentDepthMap() const;
  cv::Mat currentValidDepthMask() const;
  cv::Mat currentPointCloudMap() const;
  cv::Mat currentDisparityMap() const;
  cv::Mat currentDisparityMap32F() const;
  cv::Mat currentIrCameraImage() const;
  cv::Mat currentBgrCameraImage() const;
  cv::Mat currentGrayCameraImage() const;

  int deviceIndex() const { return mDeviceIndex; }
  void deviceIndex(int index);


public slots:

  /*
  void emitDepthMap();
  void emitValidDepthMask();
  void emitPointCloudMap();
  void emitDisparityMap();
  void emitDisparityMap32F();
  void emitIrCameraImage();
  void emitBgrCameraImage();
  void emitGrayCameraImage();
  */

signals:

  /*
  void depthMapReady();
  void validDepthMaskReady();
  void pointCloudMapReady();
  void disparityMapReady();
  void disparityMap32FReady();
  void irCameraImageReady();
  void bgrCameraImageReady();
  void grayCameraImageReady();
  */

  void depthMap(const cv::Mat& mat);
  void validDepthMask(const cv::Mat& mat);
  void pointCloudMap(const cv::Mat& mat);
  void disparityMap(const cv::Mat& mat);
  void disparityMap32F(const cv::Mat& mat);
  void irCameraImage(const cv::Mat& mat);
  void bgrCameraImage(const cv::Mat& mat);
  void grayCameraImage(const cv::Mat& mat);

protected:

  bool open(int deviceIndex);

  void close();

  void stop();

  void run() override;

  void connectNotify(const QMetaMethod &signal) override;
  void disconnectNotify(const QMetaMethod &signal) override;

  void updateConnectionState();

  void updateImages();
  void updateDepthMap();
  void updateValidDepthMask();
  void updatePointCloudMap();
  void updateDisparityMap();
  void updateDisparityMap32F();
  void updateIrCameraImage();
  void updateBGRCameraImage();
  void updateGrayCameraImage();

private:

  int mDeviceIndex;
  cv::VideoCapture mKinect;

  QMutex mMutex;
  QWaitCondition mWaitCondition;
  bool mStop;

  bool mDepthGeneratorPresent;
  bool mIrImageGeneratorPresent;
  bool mColorImageGeneratorPresent;

  cv::Mat mDepthMap;
  cv::Mat mValidDepthMask;
  cv::Mat mPointCloudMap;
  cv::Mat mDisparityMap;
  cv::Mat mDisparityMap32F;
  cv::Mat mIrCameraImage;
  cv::Mat mBGRCameraImage;
  cv::Mat mGrayCameraImage;

  bool mUpdateDepthMapRequired;
  bool mUpdateDepthMaskRequired;
  bool mUpdatePointCloudMapRequired;
  bool mUpdateDisparityMapRequired;
  bool mUpdateDisparityMap32FRequired;
  bool mUpdateIrCameraImageRequired;
  bool mUpdateBGRCameraImageRequired;
  bool mUpdateGrayCameraImageRequired;
};

#endif // KINECTIMAGESOURCE_H
