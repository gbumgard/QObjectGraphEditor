#include "OpenCvFactoryPlugin.h"
#include "KinectV2Sensor.h"

#include <QMutexLocker>
#include <QMetaObject>
#include <QMetaMethod>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <signal.h>

REGISTER_CLASS(KinectV2Sensor)

KinectV2Sensor::KinectV2Sensor(QObject* parent)
  : QThread(parent)
  , _stop(false)
  , _updateDepthMapRequired(false)
  , _updateIrCameraImageRequired(false)
  , _updateBGRCameraImageRequired(false)
  , _updatePointCloudMapRequired(false)
  , _listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
  , _undistortedDepthFrame(512, 424, 4)
  , _registeredColorFrame(512, 424, 4)
  , _bigMat(1920, 1082, 4)
  , _serial()
  , _qnan(std::numeric_limits<float>::quiet_NaN())
  , _filterUnmatchedPixels(false)
  , _generateHighDef(false)
  , _mirror(true)
{
  if (open(_serial)) {
    start();
  }
}

KinectV2Sensor::~KinectV2Sensor()
{
  qDebug() << Q_FUNC_INFO;
  stop();
  close();
}

void KinectV2Sensor::stop()
{
  qDebug() << Q_FUNC_INFO;
  if (isRunning()) {
    _stop = true;
    _waitCondition.wakeAll();
    wait();
  }
}

void KinectV2Sensor::run()
{
  _mutex.lock();
  if (!_dev) {
    _waitCondition.wait(&_mutex);
  }
  _mutex.unlock();

  while (!_stop) {
    update();
  }
}

bool KinectV2Sensor::open(const std::string& serial)
{
  QMutexLocker lock(&_mutex);
  if(_freenect2.enumerateDevices() == 0) {
    qDebug() << "no kinect2 connected!";
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_ERROR,"No Kinect2 Connected");
    return false;
  }

  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
  _logger = libfreenect2::getGlobalLogger();

  if(!serial.empty())
    _serial = serial;
  else
    _serial = _freenect2.getDefaultDeviceSerialNumber();

  _dev = nullptr;
  Processor p = OPENGL;
  switch (p)
  {
#if WITH_OPENCL
    case OPENCL:
      qDebug() << "creating OpenCL pipeline";
      _pipeline = new libfreenect2::OpenCLPacketPipeline());
      break;
#endif
    case OPENGL:
      qDebug() << "creating OpenGL processor";
      _pipeline = new libfreenect2::OpenGLPacketPipeline();
      break;
    case CUDA:
      qDebug() << "creating Cuda processor";
      _pipeline = new libfreenect2::CudaPacketPipeline();
      break;
    default:
      qDebug() << "creating Cpu processor";
      _pipeline = new libfreenect2::CpuPacketPipeline();
      break;
  }

  if(serial.empty())
    _dev = _freenect2.openDefaultDevice(_pipeline);
  else
    _dev = _freenect2.openDevice(serial,_pipeline);

  if (!_dev) {
    qDebug() << "open failed";
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_ERROR,"Open Failed");
    return false;
  }

  qDebug() << "open succeeded";
  ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");

  _dev->setColorFrameListener(&_listener);
  _dev->setIrAndDepthFrameListener(&_listener);

  if (_dev->start()) {

    qDebug() << "start succeeded on device" << _dev->getSerialNumber().c_str() << _dev->getFirmwareVersion().c_str();
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");

    _registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());

    prepareMake3D(_dev->getIrCameraParams());

    _waitCondition.wakeAll();
    return true;
  }
  else {
    qDebug() << "start failed";
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_ERROR,"Kinect failed to start");
    return false;
  }
}

void KinectV2Sensor::close()
{
  shutDown();
}

libfreenect2::Freenect2Device::IrCameraParams KinectV2Sensor::getIrParameters(){
  libfreenect2::Freenect2Device::IrCameraParams ir = _dev->getIrCameraParams();
  return ir;
}

libfreenect2::Freenect2Device::ColorCameraParams KinectV2Sensor::getRgbParameters(){
  libfreenect2::Freenect2Device::ColorCameraParams rgb = _dev->getColorCameraParams();
  return rgb;
}

void KinectV2Sensor::disableLog() {
  _logger = libfreenect2::getGlobalLogger();
  libfreenect2::setGlobalLogger(nullptr);
}

void KinectV2Sensor::enableLog() {
  libfreenect2::setGlobalLogger(_logger);
}

void KinectV2Sensor::printParameters(){
  libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
  qDebug() << "rgb fx=" << cp.fx << ",fy=" << cp.fy <<
               ",cx=" << cp.cx << ",cy=" << cp.cy;
  libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();
  qDebug() << "ir fx=" << ip.fx
            << ",fy=" << ip.fy
            << ",cx=" << ip.cx
            << ",cy=" << ip.cy
            << ",k1=" << ip.k1
            << ",k2=" << ip.k2
            << ",k3=" << ip.k3
            << ",p1=" << ip.p1
            << ",p2=" << ip.p2
           ;
}

void KinectV2Sensor::storeParameters(){

  libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
  libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();

  cv::Mat rgb = (cv::Mat_<float>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
  cv::Mat depth = (cv::Mat_<float>(3,3) << ip.fx, 0, ip.cx, 0, ip.fy, ip.cy, 0, 0, 1);
  cv::Mat depth_dist = (cv::Mat_<float>(1,5) << ip.k1, ip.k2, ip.p1, ip.p2, ip.k3);
  qDebug() << "storing " << _serial.c_str();
  cv::FileStorage fs("calib_" + _serial + ".yml", cv::FileStorage::WRITE);

  fs << "CcameraMatrix" << rgb;
  fs << "DcameraMatrix" << depth << "distCoeffs" << depth_dist;

  fs.release();
}

void KinectV2Sensor::shutDown(){
  qDebug() << Q_FUNC_INFO;
  if (_dev) {
    _dev->stop();
    _dev->close();
  }
}

libfreenect2::SyncMultiFrameListener* KinectV2Sensor::getListener(){
  return &_listener;
}

void KinectV2Sensor::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
{
  const int w = 512;
  const int h = 424;
  float * pm1 = _colmap.data();
  float * pm2 = _rowmap.data();
  for(int i = 0; i < w; i++)
  {
    *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
  }
  for (int i = 0; i < h; i++)
  {
    *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
  }
}

void KinectV2Sensor::connectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QThread::connectNotify(signal);
}

void KinectV2Sensor::disconnectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QThread::disconnectNotify(signal);
}

void KinectV2Sensor::updateConnectionState() {

  static const QMetaMethod depthMapSignal = QMetaMethod::fromSignal(&KinectV2Sensor::depth);
  static const QMetaMethod irCameraImageSignal = QMetaMethod::fromSignal(&KinectV2Sensor::ir);
  static const QMetaMethod bgrCameraImageSignal = QMetaMethod::fromSignal(&KinectV2Sensor::color);
  static const QMetaMethod undistortedDepthSignal = QMetaMethod::fromSignal(&KinectV2Sensor::undistortedDepth);
  static const QMetaMethod registeredColorSignal = QMetaMethod::fromSignal(&KinectV2Sensor::registeredColor);
#if WITH_PCL
  static const QMetaMethod pointCloudMapSignal = QMetaMethod::fromSignal(&KinectV2Sensor::pointCloud);
#endif

  _updateDepthMapRequired = isSignalConnected(depthMapSignal);
  _updateIrCameraImageRequired = isSignalConnected(irCameraImageSignal);
  _updateBGRCameraImageRequired = isSignalConnected(bgrCameraImageSignal);
  mUpdateUndistortedDepthRequired = isSignalConnected(undistortedDepthSignal);
  mUpdateRegisteredColorRequired = isSignalConnected(registeredColorSignal);
#if WITH_PCL
  mUpdatePointCloudMapRequired = isSignalConnected(pointCloudMapSignal);
#endif

}

void KinectV2Sensor::update()
{
  QMutexLocker lock(&_mutex);
  if (_updateDepthMapRequired ||
      _updateIrCameraImageRequired ||
      _updateBGRCameraImageRequired ||
      mUpdateUndistortedDepthRequired ||
      mUpdateRegisteredColorRequired ||
      _updatePointCloudMapRequired) {

    int timestamp = MatEvent::now();

    if (_listener.waitForNewFrame(_frames,1000)) {
      libfreenect2::Frame * rgbFrame = _frames[libfreenect2::Frame::Color];
      libfreenect2::Frame * depthFrame = _frames[libfreenect2::Frame::Depth];
      libfreenect2::Frame * irFrame = _frames[libfreenect2::Frame::Ir];

      cv::Mat depthMat;
      cv::Mat colorMat;

      cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).copyTo(depthMat);
      cv::Mat(rgbFrame->height, rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(colorMat);

      if (_updateIrCameraImageRequired) {
        cv::Mat irMat;
        cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data).copyTo(irMat);
        if (_mirror == true) {
          cv::flip(irMat, irMat, 1);
        }
        emit ir(QVariant::fromValue(MatEvent(irMat,timestamp)));
        emit mat(QVariant::fromValue(irMat));
      }

      if (mUpdateUndistortedDepthRequired || mUpdateRegisteredColorRequired || _updatePointCloudMapRequired) {

        cv::Mat undistortedDepthMat;
        cv::Mat registeredColorMat;

        _registration->apply(rgbFrame, depthFrame, &_undistortedDepthFrame, &_registeredColorFrame, _filterUnmatchedPixels, &_bigMat, _map);

        cv::Mat(_undistortedDepthFrame.height, _undistortedDepthFrame.width, CV_32FC1, _undistortedDepthFrame.data).copyTo(undistortedDepthMat);
        cv::Mat(_registeredColorFrame.height, _registeredColorFrame.width, CV_8UC4, _registeredColorFrame.data).copyTo(registeredColorMat);

        if (mUpdateUndistortedDepthRequired || mUpdateRegisteredColorRequired) {
          cv::cvtColor(registeredColorMat,registeredColorMat,cv::COLOR_RGBA2BGR,3);

          if (_mirror == true) {
            cv::flip(undistortedDepthMat, undistortedDepthMat, 1);
            cv::flip(registeredColorMat, registeredColorMat, 1);
          }

          cv::Mat mat16UC1;
          undistortedDepthMat.convertTo(mat16UC1,CV_16UC1,1.0);

          emit undistortedDepth(QVariant::fromValue(MatEvent(mat16UC1,timestamp)));
          emit registeredColor(QVariant::fromValue(MatEvent(registeredColorMat,timestamp)));
          emit mat(QVariant::fromValue(mat16UC1));
        }

      }

      _listener.release(_frames);

      if (_mirror == true) {
        cv::flip(depthMat, depthMat, 1);
        cv::flip(colorMat, colorMat, 1);
      }

      cv::Mat mat16UC1;
      depthMat.convertTo(mat16UC1,CV_16UC1,1.0);

      cv::cvtColor(colorMat,colorMat,cv::COLOR_RGBA2BGR,3);

      emit depth(QVariant::fromValue(MatEvent(mat16UC1,timestamp)));
      emit color(QVariant::fromValue(MatEvent(colorMat,timestamp)));

  #if WITH_PCL

      const std::size_t w = mUndistortedDepthFrame.width;
      const std::size_t h = mUndistortedDepthFrame.height;

      if (mUpdatePointCloudMapRequired) {
        cv::Mat tmp_itD0(mUndistortedDepthFrame.height, mUndistortedDepthFrame.width, CV_8UC4, mUndistortedDepthFrame.data);
        cv::Mat tmp_itRGB0(mRegisteredColorFrame.height, mRegisteredColorFrame.width, CV_8UC4, mRegisteredColorFrame.data);

        if (!mPointCloudMap) {
          mPointCloudMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));
        }
        else if(mPointCloudMap->size() != w * h) {
          mPointCloudMap->resize(w * h);
        }

        if (mirror_ == true){
          cv::flip(tmp_itD0,tmp_itD0,1);
          cv::flip(tmp_itRGB0,tmp_itRGB0,1);
        }

        const float * itD0 = (float *) tmp_itD0.ptr();
        const char * itRGB0 = (char *) tmp_itRGB0.ptr();

        pcl::PointXYZRGB * itP = &mPointCloudMap->points[0];
        bool is_dense = true;

        for(std::size_t y = 0; y < h; ++y){

          const unsigned int offset = y * w;
          const float * itD = itD0 + offset;
          const char * itRGB = itRGB0 + offset * 4;
          const float dy = rowmap(y);

          for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
          {
            const float depth_value = *itD / 1000.0f;

            if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){

              const float rx = colmap(x) * depth_value;
              const float ry = dy * depth_value;
              itP->z = depth_value;
              itP->x = rx;
              itP->y = ry;

              itP->b = itRGB[0];
              itP->g = itRGB[1];
              itP->r = itRGB[2];
            }
            else {
              itP->z = qnan_;
              itP->x = qnan_;
              itP->y = qnan_;

              itP->b = qnan_;
              itP->g = qnan_;
              itP->r = qnan_;
              is_dense = false;
            }
          }
        }

        mPointCloudMap->is_dense = is_dense;

        emit pointCloud(mPointCloudMap);

      }
  #endif
    }
  }
}
