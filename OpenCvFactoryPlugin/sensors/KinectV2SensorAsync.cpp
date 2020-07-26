#include "OpenCvFactoryPlugin.h"
#include "KinectV2SensorAsync.h"

#include <QMutexLocker>
#include <QMetaObject>
#include <QMetaMethod>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <signal.h>
#include <set>

REGISTER_CLASS(KinectV2SensorAsync)

std::set<std::string> KinectV2SensorAsync::_openDeviceSerialNumbers;

KinectV2SensorAsync::KinectV2SensorAsync(QObject* parent)
  : QObject(parent)
  , libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
  , _updateDepthMapRequired(false)
  , _updateIrCameraImageRequired(false)
  , _updateBGRCameraImageRequired(false)
  , _updateUndistortedDepthRequired(false)
  , _updateRegisteredColorRequired(false)
  , _updatePointCloudMapRequired(false)
  , _dev(nullptr)
  , _pipeline(nullptr)
  , _registration(nullptr)
  , _colorFrame(nullptr)
  , _irFrame(nullptr)
  , _depthFrame(nullptr)
  , _undistortedDepthFrame(512, 424, 4)
  , _registeredColorFrame(512, 424, 4)
  , _bigDepthFrame(1920, 1082, 4)
  , _serialNumber()
  , _qnan(std::numeric_limits<float>::quiet_NaN())
  , _filterUnmatched(false)
  , _generateHighDef(false)
  , _mirror(true)
  , _colorFormat(COLOR_RGB)
{
  open();
}

KinectV2SensorAsync::~KinectV2SensorAsync()
{
  qDebug() << Q_FUNC_INFO;
  close();
}

void KinectV2SensorAsync::close(){
  qDebug() << Q_FUNC_INFO;

  if (_dev) {
    qDebug() << "stopping the device";
    _dev->stop();
    qDebug() << "closing the device";
    _dev->close();
    qDebug() << "deleting the device";
    delete _dev;
    _dev = nullptr;
  }

  _openDeviceSerialNumbers.erase(_serialNumber);

  qDebug() << "deleting the registration object";
  delete _registration;
  _registration = nullptr;

  qDebug() << "deleting the color frame";
  delete _colorFrame;
  _colorFrame = nullptr;

  qDebug() << "deleting the ir frame";
  delete _irFrame;
  _irFrame = nullptr;

  qDebug() << "deleting the depth frame";
  delete _depthFrame;
  _depthFrame = nullptr;
}

bool KinectV2SensorAsync::open()
{
  qDebug() << Q_FUNC_INFO;

  int count = _freenect2.enumerateDevices();
  if (count == 0) {
    qDebug() << "no kinect2 devices found";
    return false;
  }

  int deviceIndex = -1;

  // Check for open devices first so we don't attempt to open the same device again (bad).
  if (_openDeviceSerialNumbers.size() != 0) {
    for (int i=0; i<count; i++) {
      std::string serialNumber = _freenect2.getDeviceSerialNumber(i);
      if (_openDeviceSerialNumbers.count(serialNumber) == 0) {
        qDebug() << "device" << i << serialNumber.c_str() << "is available for use";
        deviceIndex = i;
        break;
      }
      else {
        qDebug() << "device" << i << serialNumber.c_str() << "is already in use";
      }
    }
  }
  else {
    deviceIndex = 0;
  }

  if (deviceIndex == -1) {
    qDebug() << "all available Kinect2 devices are in use";
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_ERROR,"No Device");
    return false;
  }

  libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
  _logger = libfreenect2::getGlobalLogger();

  _serialNumber = _freenect2.getDeviceSerialNumber(deviceIndex);

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
      qDebug() << "creating OpenGL pipeline";
      _pipeline = new libfreenect2::OpenGLPacketPipeline();
      break;
    case CUDA:
      qDebug() << "creating Cuda pipeline";
      _pipeline = new libfreenect2::CudaPacketPipeline();
      break;
    default:
      qDebug() << "creating Cpu pipeline";
      _pipeline = new libfreenect2::CpuPacketPipeline();
      break;
  }

  qDebug() << "attach pipeline to KinectV2 device" << _serialNumber.c_str();

  _dev = _freenect2.openDevice(deviceIndex,_pipeline);

  if (!_dev) {
    qDebug() << "open failed";
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_ERROR,"Device Error");
    return false;
  }

  // Add the newly opened devide to the set.
  _openDeviceSerialNumbers.insert(_serialNumber);

  qDebug() << "open succeeded";
  ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");

  _dev->setColorFrameListener(this);
  _dev->setIrAndDepthFrameListener(this);

  qDebug() << "starting KinectV2 device";

  if (_dev->start()) {

    qDebug() << "start succeeded on device" << _dev->getSerialNumber().c_str() << _dev->getFirmwareVersion().c_str();
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");

    _registration = new libfreenect2::Registration(_dev->getIrCameraParams(), _dev->getColorCameraParams());

    prepareMake3D(_dev->getIrCameraParams());

    return true;
  }
  else {

    qDebug() << "start failed";
    close();
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_ERROR,"Device Error");
    return false;
  }
}

libfreenect2::Freenect2Device::IrCameraParams KinectV2SensorAsync::getIrParameters(){
  libfreenect2::Freenect2Device::IrCameraParams ir = _dev->getIrCameraParams();
  return ir;
}

libfreenect2::Freenect2Device::ColorCameraParams KinectV2SensorAsync::getRgbParameters(){
  libfreenect2::Freenect2Device::ColorCameraParams rgb = _dev->getColorCameraParams();
  return rgb;
}

void KinectV2SensorAsync::disableLog() {
  _logger = libfreenect2::getGlobalLogger();
  libfreenect2::setGlobalLogger(nullptr);
}

void KinectV2SensorAsync::enableLog() {
  libfreenect2::setGlobalLogger(_logger);
}

void KinectV2SensorAsync::printParameters(){
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

void KinectV2SensorAsync::storeParameters(){

  libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
  libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();

  cv::Mat rgb = (cv::Mat_<float>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
  cv::Mat depth = (cv::Mat_<float>(3,3) << ip.fx, 0, ip.cx, 0, ip.fy, ip.cy, 0, 0, 1);
  cv::Mat depth_dist = (cv::Mat_<float>(1,5) << ip.k1, ip.k2, ip.p1, ip.p2, ip.k3);
  qDebug() << "storing " << _serialNumber.c_str();
  cv::FileStorage fs("calib_" + _serialNumber + ".yml", cv::FileStorage::WRITE);

  fs << "CcameraMatrix" << rgb;
  fs << "DcameraMatrix" << depth << "distCoeffs" << depth_dist;

  fs.release();
}

void KinectV2SensorAsync::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
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

void KinectV2SensorAsync::connectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QObject::connectNotify(signal);
}

void KinectV2SensorAsync::disconnectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QObject::disconnectNotify(signal);
}

void KinectV2SensorAsync::updateConnectionState() {

  static const QMetaMethod depthMapSignal = QMetaMethod::fromSignal(&KinectV2SensorAsync::depth);
  static const QMetaMethod irCameraImageSignal = QMetaMethod::fromSignal(&KinectV2SensorAsync::ir);
  static const QMetaMethod bgrCameraImageSignal = QMetaMethod::fromSignal(&KinectV2SensorAsync::color);
  static const QMetaMethod undistortedDepthSignal = QMetaMethod::fromSignal(&KinectV2SensorAsync::undistorted);
  static const QMetaMethod registeredColorSignal = QMetaMethod::fromSignal(&KinectV2SensorAsync::registered);
#if WITH_PCL
  static const QMetaMethod pointCloudMapSignal = QMetaMethod::fromSignal(&KinectV2SensorAsync::pointCloud);
#endif

  _updateDepthMapRequired = isSignalConnected(depthMapSignal);
  _updateIrCameraImageRequired = isSignalConnected(irCameraImageSignal);
  _updateBGRCameraImageRequired = isSignalConnected(bgrCameraImageSignal);
  _updateUndistortedDepthRequired = isSignalConnected(undistortedDepthSignal);
  _updateRegisteredColorRequired = isSignalConnected(registeredColorSignal);
#if WITH_PCL
  mUpdatePointCloudMapRequired = isSignalConnected(pointCloudMapSignal);
#endif

}

bool KinectV2SensorAsync::onNewFrame(libfreenect2::Frame::Type type, libfreenect2::Frame* frame) {

  QMutexLocker locker(&_mutex);

  if (frame->status != 0 ||
      frame->format == libfreenect2::Frame::Invalid ||
      frame->data == nullptr) {
    qWarning() << Q_FUNC_INFO;
    qWarning() << "received bad frame with"
               << "sequence:" << frame->sequence
               << "timestamp:" << frame->timestamp
               << "status:" << frame->status
               << "format:" << frame->format;
    delete frame;
    return true;
  }

  int timestamp = MatEvent::now();

  if (type == libfreenect2::Frame::Type::Color) {

    if (_updateBGRCameraImageRequired) {

      cv::Mat colorMat;
      cv::Mat(frame->height, frame->width, CV_8UC4, frame->data).copyTo(colorMat);

      if (_mirror == true) {
        cv::flip(colorMat, colorMat, 1);
      }

      if (frame->format == libfreenect2::Frame::BGRX) {
        if (_colorFormat == COLOR_RGB) {
          cv::cvtColor(colorMat,colorMat,cv::COLOR_BGRA2RGB,3);
        }
        else {
          cv::cvtColor(colorMat,colorMat,cv::COLOR_BGRA2BGR,3);
        }
      }
      else if (frame->format == libfreenect2::Frame::RGBX) {
        if (_colorFormat == COLOR_BGR) {
          cv::cvtColor(colorMat,colorMat,cv::COLOR_RGBA2BGR,3);
        }
        else {
          cv::cvtColor(colorMat,colorMat,cv::COLOR_RGBA2RGB,3);
        }
      }

      emit color(QVariant::fromValue(MatEvent(colorMat,timestamp)));

    }
  }
  else if (frame->sequence != _currentSequenceNumber) {

    // The new frame has a new sequence number (in the IR and DEEPTH frame seequence).
    // Process the frames captured under the previous sequence number before storing the new one.

    if (_irFrame) {

      if (_updateIrCameraImageRequired) {

        cv::Mat irMat;
        cv::Mat(_irFrame->height, _irFrame->width, CV_32FC1, _irFrame->data).copyTo(irMat);

        if (_mirror == true) {
          cv::flip(irMat, irMat, 1);
        }

        emit ir(QVariant::fromValue(MatEvent(irMat,timestamp)));
      }

    }

    if (_depthFrame) {

      if (_updateDepthMapRequired) {

        cv::Mat depthMat;
        cv::Mat(_depthFrame->height, _depthFrame->width, CV_32FC1, _depthFrame->data).copyTo(depthMat);

        if (_mirror == true) {
          cv::flip(depthMat, depthMat, 1);
        }

        cv::Mat mat16UC1;
        depthMat.convertTo(mat16UC1,CV_16UC1,1.0);

        emit depth(QVariant::fromValue(MatEvent(depthMat,timestamp)));
      }

    }


    if (_colorFrame && _depthFrame &&
        (_updateUndistortedDepthRequired ||
         _updateRegisteredColorRequired ||
         _updatePointCloudMapRequired)) {


      // TODO: generate big depth map output
#if 0
      qDebug() << "BEFORE"
               << "COLOR:"
               << "sequence:" << _colorFrame->sequence
               << "timestamp:" << _colorFrame->timestamp
               << "status:" << _colorFrame->status
               << "format:" << _colorFrame->format
               << "DEPTH:"
               << "sequence:" << _depthFrame->sequence
               << "timestamp:" << _depthFrame->timestamp
               << "status:" << _depthFrame->status
               << "format:" << _depthFrame->format
               << "UNDISTORTED"
               << "sequence:" << _undistortedDepthFrame.sequence
               << "timestamp:" << _undistortedDepthFrame.timestamp
               << "status:" << _undistortedDepthFrame.status
               << "format:" << _undistortedDepthFrame.format
               << "REGISTERED:"
               << "sequence:" << _registeredColorFrame.sequence
               << "timestamp:" << _registeredColorFrame.timestamp
               << "status:" << _registeredColorFrame.status
               << "format:" << _registeredColorFrame.format;
#endif

      libfreenect2::Frame undistortedDepthFrame(512, 424, 4);
      libfreenect2::Frame registeredColorFrame(512, 424, 4);

      _registration->apply(_colorFrame,
                           _depthFrame,
                           &undistortedDepthFrame,
                           &registeredColorFrame);

#if 0
      _registration->apply(_colorFrame,
                           _depthFrame,
                           &undistortedDepthFrame,
                           &registeredColorFrame,
                           _filterUnmatched,
                           &_bigDepthFrame,
                           _map);
      qDebug() << "AFTER"
               << "COLOR:"
               << "sequence:" << _colorFrame->sequence
               << "timestamp:" << _colorFrame->timestamp
               << "status:" << _colorFrame->status
               << "format:" << _colorFrame->format
               << "DEPTH:"
               << "sequence:" << _depthFrame->sequence
               << "timestamp:" << _depthFrame->timestamp
               << "status:" << _depthFrame->status
               << "format:" << _depthFrame->format
               << "UNDISTORTED"
               << "sequence:" << _undistortedDepthFrame.sequence
               << "timestamp:" << _undistortedDepthFrame.timestamp
               << "status:" << _undistortedDepthFrame.status
               << "format:" << _undistortedDepthFrame.format
               << "REGISTERED:"
               << "sequence:" << _registeredColorFrame.sequence
               << "timestamp:" << _registeredColorFrame.timestamp
               << "status:" << _registeredColorFrame.status
               << "format:" << _registeredColorFrame.format;
#endif

      if (_updateUndistortedDepthRequired) {

        cv::Mat undistortedDepthMat;

        //cv::Mat(_undistortedDepthFrame.height, _undistortedDepthFrame.width, CV_32FC1, _undistortedDepthFrame.data).copyTo(undistortedDepthMat);
        cv::Mat(undistortedDepthFrame.height, undistortedDepthFrame.width, CV_32FC1, undistortedDepthFrame.data).copyTo(undistortedDepthMat);

        if (_mirror == true) {
          cv::flip(undistortedDepthMat, undistortedDepthMat, 1);
        }

        cv::Mat mat16UC1;
        undistortedDepthMat.convertTo(mat16UC1,CV_16UC1,1.0);

        emit undistorted(QVariant::fromValue(MatEvent(mat16UC1,timestamp)));

      }

      if (_updateRegisteredColorRequired) {

        cv::Mat registeredColorMat;

        //cv::Mat(_registeredColorFrame.height, _registeredColorFrame.width, CV_8UC4, _registeredColorFrame.data).copyTo(registeredColorMat);
        cv::Mat(registeredColorFrame.height, registeredColorFrame.width, CV_8UC4, registeredColorFrame.data).copyTo(registeredColorMat);

        if (_colorFrame->format == libfreenect2::Frame::BGRX) {
          if (_colorFormat == COLOR_RGB) {
            cv::cvtColor(registeredColorMat,registeredColorMat,cv::COLOR_BGRA2RGB,3);
          }
          else {
            cv::cvtColor(registeredColorMat,registeredColorMat,cv::COLOR_BGRA2BGR,3);
          }
        }
        else if (_colorFrame->format == libfreenect2::Frame::RGBX) {
          if (_colorFormat == COLOR_BGR) {
            cv::cvtColor(registeredColorMat,registeredColorMat,cv::COLOR_RGBA2BGR,3);
          }
          else {
            cv::cvtColor(registeredColorMat,registeredColorMat,cv::COLOR_RGBA2RGB,3);
          }
        }

        if (_mirror == true) {
          cv::flip(registeredColorMat, registeredColorMat, 1);
        }

        emit registered(QVariant::fromValue(MatEvent(registeredColorMat,timestamp)));
      }

      if (_updatePointCloudMapRequired) {

        const std::size_t w = _undistortedDepthFrame.width;
        const std::size_t h = _undistortedDepthFrame.height;

        if (_updatePointCloudMapRequired) {
          //cv::Mat tmp_itD0(_undistortedDepthFrame.height, _undistortedDepthFrame.width, CV_8UC4, _undistortedDepthFrame.data);
          //cv::Mat tmp_itRGB0(_registeredColorFrame.height, _registeredColorFrame.width, CV_8UC4, _registeredColorFrame.data);
          cv::Mat tmp_itD0(undistortedDepthFrame.height, undistortedDepthFrame.width, CV_8UC4, undistortedDepthFrame.data);
          cv::Mat tmp_itRGB0(registeredColorFrame.height, registeredColorFrame.width, CV_8UC4, registeredColorFrame.data);

          if (_pointCloudMap.empty()) {
            _pointCloudMap = pcl::PointCloud<pcl::PointXYZRGB>(w, h);
          }
          else if(_pointCloudMap.size() != w * h) {
            _pointCloudMap.resize(w * h);
          }

          if (_mirror == true){
            cv::flip(tmp_itD0,tmp_itD0,1);
            cv::flip(tmp_itRGB0,tmp_itRGB0,1);
          }

          const float * itD0 = (float *) tmp_itD0.ptr();
          const char * itRGB0 = (char *) tmp_itRGB0.ptr();

          pcl::PointXYZRGB * itP = &_pointCloudMap.points[0];
          bool is_dense = true;

          for(std::size_t y = 0; y < h; ++y){

            const unsigned int offset = y * w;
            const float * itD = itD0 + offset;
            const char * itRGB = itRGB0 + offset * 4;
            const float dy = _rowmap(y);

            for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
            {
              const float depth_value = *itD / 1000.0f;

              if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){

                const float rx = _colmap(x) * depth_value;
                const float ry = dy * depth_value;
                itP->z = depth_value;
                itP->x = rx;
                itP->y = ry;

                itP->b = itRGB[0];
                itP->g = itRGB[1];
                itP->r = itRGB[2];
              }
              else {
                itP->z = _qnan;
                itP->x = _qnan;
                itP->y = _qnan;

                itP->b = _qnan;
                itP->g = _qnan;
                itP->r = _qnan;
                is_dense = false;
              }
            }
          }

          _pointCloudMap.is_dense = is_dense;

          emit pcl(QVariant::fromValue(_pointCloudMap));

        }
      }
    }

    // Delete stored Ir and Depth frames.
    // Null pointer value are used to indicate missing frames for the new sequence number.

    delete _irFrame;
    _irFrame = nullptr;

    delete _depthFrame;
    _depthFrame = nullptr;

    _currentSequenceNumber = frame->sequence;

  }

  // We always store the new frame, replacing any that might
  // have been stored under the current sequence number.

  if (type == libfreenect2::Frame::Type::Depth) {
    delete _depthFrame;
    _depthFrame = frame;
  }
  else if (type == libfreenect2::Frame::Type::Ir) {
    delete _irFrame;
    _irFrame = frame;
  }
  else if (type == libfreenect2::Frame::Type::Color) {
    delete _colorFrame;
    _colorFrame = frame;
  }
  else {
    // An unexpected frame type. Just delete it.
    delete frame;
  }

  return true;

}
