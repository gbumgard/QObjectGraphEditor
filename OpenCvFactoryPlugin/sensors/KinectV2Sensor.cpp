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
  , mStop(false)
  , mUpdateDepthMapRequired(false)
  , mUpdateIrCameraImageRequired(false)
  , mUpdateBGRCameraImageRequired(false)
  , mUpdatePointCloudMapRequired(false)
  , listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
  , mUndistortedDepthFrame(512, 424, 4)
  , mRegisteredColorFrame(512, 424, 4)
  , big_mat_(1920, 1082, 4)
  , serial_()
  , qnan_(std::numeric_limits<float>::quiet_NaN())
  , mFilterUnmatchedPixels(false)
  , mGenerateHighDef(false)
  , mirror_(true)
{
  open(serial_);
  start();
}

KinectV2Sensor::~KinectV2Sensor()
{
  stop();
  close();
}

void KinectV2Sensor::stop()
{
  mStop = true;
  mWaitCondition.wakeAll();
  wait();
}

void KinectV2Sensor::run()
{
  mMutex.lock();
  if (!dev_) {
    mWaitCondition.wait(&mMutex);
  }
  mMutex.unlock();

  while (!mStop) {
    update();
  }
}

bool KinectV2Sensor::open(const std::string& serial)
{
  QMutexLocker lock(&mMutex);
  if(freenect2_.enumerateDevices() == 0) {
    std::cout << "no kinect2 connected!" << std::endl;
    exit(-1);
  }
  Processor p = OPENGL;
  switch (p)
  {
#if WITH_OPENCL
    case OPENCL:
      std::cout << "creating OpenCL processor" << std::endl;
      if(serial.empty())
        dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
      else
        dev_ = freenect2_.openDevice(serial, new libfreenect2::OpenCLPacketPipeline());
      break;
#endif
    case OPENGL:
      std::cout << "creating OpenGL processor" << std::endl;
      if (serial.empty())
        dev_ = freenect2_.openDefaultDevice (new libfreenect2::OpenGLPacketPipeline ());
      else
        dev_ = freenect2_.openDevice (serial, new libfreenect2::OpenGLPacketPipeline ());
      break;
#if WITH_CUDA
    case CUDA:
      std::cout << "creating Cuda processor" << std::endl;
      if(serial.empty())
        dev_ = freenect2_.openDefaultDevice(new libfreenect2::CudaPacketPipeline());
      else
        dev_ = freenect2_.openDevice(serial, new libfreenect2::CudaPacketPipeline());
      break;
#endif
    default:
      std::cout << "creating Cpu processor" << std::endl;
      if (serial_.empty())
        dev_ = freenect2_.openDefaultDevice (new libfreenect2::CpuPacketPipeline ());
      else
        dev_ = freenect2_.openDevice (serial, new libfreenect2::CpuPacketPipeline ());
      break;
  }

  if (!dev_) {
    std::cout << "open failed" << std::endl;
    exit(-1);
  }

  std::cout << "open succeeded" << std::endl;

  if(!serial.empty())
    serial_ = serial;
  else
    serial_ = freenect2_.getDefaultDeviceSerialNumber();

  dev_->setColorFrameListener(&listener_);
  dev_->setIrAndDepthFrameListener(&listener_);

  if (dev_->start()) {

    logger_ = libfreenect2::getGlobalLogger();

    registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

    prepareMake3D(dev_->getIrCameraParams());

    mWaitCondition.wakeAll();
    return true;
  }
  else {
    mStop = true;
    mWaitCondition.wakeAll();
  }
  return false;
}

void KinectV2Sensor::close()
{
  shutDown();
}

libfreenect2::Freenect2Device::IrCameraParams KinectV2Sensor::getIrParameters(){
  libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
  return ir;
}

libfreenect2::Freenect2Device::ColorCameraParams KinectV2Sensor::getRgbParameters(){
  libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
  return rgb;
}

void KinectV2Sensor::disableLog() {
  logger_ = libfreenect2::getGlobalLogger();
  libfreenect2::setGlobalLogger(nullptr);
}

void KinectV2Sensor::enableLog() {
  libfreenect2::setGlobalLogger(logger_);
}

void KinectV2Sensor::printParameters(){
  libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
  std::cout << "rgb fx=" << cp.fx << ",fy=" << cp.fy <<
               ",cx=" << cp.cx << ",cy=" << cp.cy << std::endl;
  libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();
  std::cout << "ir fx=" << ip.fx
            << ",fy=" << ip.fy
            << ",cx=" << ip.cx
            << ",cy=" << ip.cy
            << ",k1=" << ip.k1
            << ",k2=" << ip.k2
            << ",k3=" << ip.k3
            << ",p1=" << ip.p1
            << ",p2=" << ip.p2
            << std::endl;
}

void KinectV2Sensor::storeParameters(){

  libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
  libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();

  cv::Mat rgb = (cv::Mat_<float>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
  cv::Mat depth = (cv::Mat_<float>(3,3) << ip.fx, 0, ip.cx, 0, ip.fy, ip.cy, 0, 0, 1);
  cv::Mat depth_dist = (cv::Mat_<float>(1,5) << ip.k1, ip.k2, ip.p1, ip.p2, ip.k3);
  std::cout << "storing " << serial_ << std::endl;
  cv::FileStorage fs("calib_" + serial_ + ".yml", cv::FileStorage::WRITE);

  fs << "CcameraMatrix" << rgb;
  fs << "DcameraMatrix" << depth << "distCoeffs" << depth_dist;

  fs.release();
}

void KinectV2Sensor::shutDown(){
  dev_->stop();
  dev_->close();
}

libfreenect2::SyncMultiFrameListener* KinectV2Sensor::getListener(){
  return &listener_;
}

void KinectV2Sensor::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
{
  const int w = 512;
  const int h = 424;
  float * pm1 = colmap.data();
  float * pm2 = rowmap.data();
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

  mUpdateDepthMapRequired = isSignalConnected(depthMapSignal);
  mUpdateIrCameraImageRequired = isSignalConnected(irCameraImageSignal);
  mUpdateBGRCameraImageRequired = isSignalConnected(bgrCameraImageSignal);
  mUpdateUndistortedDepthRequired = isSignalConnected(undistortedDepthSignal);
  mUpdateRegisteredColorRequired = isSignalConnected(registeredColorSignal);
#if WITH_PCL
  mUpdatePointCloudMapRequired = isSignalConnected(pointCloudMapSignal);
#endif

}

void KinectV2Sensor::update()
{
  QMutexLocker lock(&mMutex);
  if (mUpdateDepthMapRequired ||
      mUpdateIrCameraImageRequired ||
      mUpdateBGRCameraImageRequired ||
      mUpdateUndistortedDepthRequired ||
      mUpdateRegisteredColorRequired ||
      mUpdatePointCloudMapRequired) {

    if (listener_.waitForNewFrame(frames_,1000)) {
      libfreenect2::Frame * rgbFrame = frames_[libfreenect2::Frame::Color];
      libfreenect2::Frame * depthFrame = frames_[libfreenect2::Frame::Depth];
      libfreenect2::Frame * irFrame = frames_[libfreenect2::Frame::Ir];

      cv::Mat depthMat;
      cv::Mat colorMat;

      cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).copyTo(depthMat);
      cv::Mat(rgbFrame->height, rgbFrame->width, CV_8UC4, rgbFrame->data).copyTo(colorMat);

      if (mUpdateIrCameraImageRequired) {
        cv::Mat irMat;
        cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data).copyTo(irMat);
        if (mirror_ == true) {
          cv::flip(irMat, irMat, 1);
        }
        emit ir(irMat);
      }

      if (mUpdateUndistortedDepthRequired || mUpdateRegisteredColorRequired || mUpdatePointCloudMapRequired) {

        cv::Mat undistortedDepthMat;
        cv::Mat registeredColorMat;

        registration_->apply(rgbFrame, depthFrame, &mUndistortedDepthFrame, &mRegisteredColorFrame, mFilterUnmatchedPixels, &big_mat_, map_);

        cv::Mat(mUndistortedDepthFrame.height, mUndistortedDepthFrame.width, CV_32FC1, mUndistortedDepthFrame.data).copyTo(undistortedDepthMat);
        cv::Mat(mRegisteredColorFrame.height, mRegisteredColorFrame.width, CV_8UC4, mRegisteredColorFrame.data).copyTo(registeredColorMat);

        if (mUpdateUndistortedDepthRequired || mUpdateRegisteredColorRequired) {
          cv::cvtColor(registeredColorMat,registeredColorMat,cv::COLOR_RGBA2BGR,3);

          if (mirror_ == true) {
            cv::flip(undistortedDepthMat, undistortedDepthMat, 1);
            cv::flip(registeredColorMat, registeredColorMat, 1);
          }

          cv::Mat mat16UC1;
          undistortedDepthMat.convertTo(mat16UC1,CV_16UC1,1.0);

          emit undistortedDepth(mat16UC1);
          emit registeredColor(registeredColorMat);
        }

      }

      listener_.release(frames_);

      if (mirror_ == true) {
        cv::flip(depthMat, depthMat, 1);
        cv::flip(colorMat, colorMat, 1);
      }

      cv::Mat mat16UC1;
      depthMat.convertTo(mat16UC1,CV_16UC1,1.0);

      cv::cvtColor(colorMat,colorMat,cv::COLOR_RGBA2BGR,3);

      emit depth(mat16UC1);
      emit color(colorMat);

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
