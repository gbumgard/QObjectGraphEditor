#ifndef OPENNISENSORSOURCE_H
#define OPENNISENSORSOURCE_H

#include <QObject>
#include <opencv2/core.hpp>
#include "OpenNiSensor.h"

class OpenNiSensorSource : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("datasource","true")
  Q_CLASSINFO("class-alias","OpenNI Sensor")
  Q_CLASSINFO("directory","Sensors")

public:

  Q_INVOKABLE explicit OpenNiSensorSource(QObject* parent = nullptr);

public slots:

  void start();

signals:

  void depthMapReady();
  void validDepthMaskReady();
  void disparityMapReady();
  void disparityMap32FReady();
  void pointCloudMapReady();
  void irCameraImageReady();
  void bgrCameraImageReady();
  void grayCameraImageReady();

  void depthMap(const cv::Mat& depthMap);
  void validDepthMaskReady(const cv::Mat& mask);
  void disparityMapReady(const cv::Mat& depthMap);
  void disparityMap32FReady(const cv::Mat& depthMap);
  void pointCloudMapReady(const cv::Mat& cloud);
  void irCameraImageReady(const cv::Mat& image);
  void bgrCameraImageReady(const cv::Mat& image);
  void grayCameraImageReady(const cv::Mat& image);

private slots:

  void onDepthMapReady();
  void onValidDepthMaskReady();
  void onDisparityMapReady();
  void onDisparityMap32FReady();
  void onPointCloudMapReady();
  void onIrCameraImageReady();
  void onBGRCameraImageReady();
  void onGrayCameraImageReady();

private:

  OpenNiSensor _sensor;

  cv::Mat _depthMap;
  cv::Mat _validDepthMask;
  cv::Mat _disparityMap;
  cv::Mat _disparityMap32F;
  cv::Mat _pointCloudMap;
  cv::Mat _irImage;
  cv::Mat _bgrImage;
  cv::Mat _grayImage;

  int _portCount;
  int _depthMapIndex;
  int _validDepthMaskIndex;
  int _disparityMapIndex;
  int _disparityMap32FIndex;
  int _pointCloudMapIndex;
  int _irImageIndex;
  int _bgrImageIndex;
  int _grayImageIndex;
};

#endif // OPENNISENSORSOURCE_H
