#include "KinectV1Sensor.h"
#include "OpenCvFactoryPlugin.h"

#include <QMutex>
#include <QMutexLocker>
#include "libfreenect/libfreenect.hpp"
#include <opencv2/imgproc.hpp>

#include <QDebug>
#include <vector>

REGISTER_CLASS(KinectV1Sensor)

typedef Freenect::FreenectDevice FreenectDevice;

class FreenectDeviceImpl
  : public FreenectDevice {
public:

  FreenectDeviceImpl(freenect_context *_ctx, int _index)
    : Freenect::FreenectDevice(_ctx, _index)
    , _cameraFrameSeqNum(0)
    , _depthFrameSeqNum(0)
    , _lastCameraFrameSeqNum(-1)
    , _lastDepthFrameSeqNum(-1)
    , _depthFrame(cv::Size(640,480),CV_16UC1, cv::Scalar(0))
    , _rgbFrame(cv::Size(640,480), CV_8UC3, cv::Scalar(0))
    , _irFrame8UC1(cv::Size(640,480), CV_8UC1, cv::Scalar(0))
    , _irFrame16UC1(cv::Size(640,480), CV_16UC1, cv::Scalar(0))
  {
  }

  virtual ~FreenectDeviceImpl() {}

  void VideoCallback(void* _frame, uint32_t) {
    QMutexLocker lock(&_colorFrameMutex);
    switch(getVideoFormat()) {
      case FREENECT_VIDEO_RGB:
      {
        uint8_t* rgb = static_cast<uint8_t*>(_frame);
        cv::Mat mat(cv::Size(640,480),CV_8UC3,rgb);
        cv::cvtColor(mat,_rgbFrame,cv::COLOR_RGB2BGR,3);
      }
      break;
      case FREENECT_VIDEO_YUV_RGB:
      {
        uint8_t* rgb = static_cast<uint8_t*>(_frame);
        cv::Mat mat(cv::Size(640,480),CV_8UC3,rgb);
        cv::cvtColor(mat,_rgbFrame,cv::COLOR_YUV2BGR,3);
      }
      break;
      case FREENECT_VIDEO_IR_8BIT:
      {
        uint8_t* ir = static_cast<uint8_t*>(_frame);
        _irFrame8UC1 = cv::Mat(cv::Size(640,480),CV_8UC1,ir);
      }
      break;
      case FREENECT_VIDEO_IR_10BIT:
      {
        uint8_t* ir = static_cast<uint8_t*>(_frame);
        _irFrame16UC1 = cv::Mat(cv::Size(640,480),CV_16UC1,ir);
      }
      break;
      default:
        break;
    }
    _cameraFrameSeqNum++;
  }

  void DepthCallback(void* _depth, uint32_t) {
    QMutexLocker lock(&_depthFrameMutex);
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    cv::Mat mat(cv::Size(640,480),CV_16UC1,depth);
    mat.copyTo(_depthFrame);
    _depthFrameSeqNum++;
  }

  bool getCameraFrame(cv::Mat& mat) {
    QMutexLocker lock(&_colorFrameMutex);
    if (_lastCameraFrameSeqNum == _cameraFrameSeqNum)
      return false;

    switch(getVideoFormat()) {
      case FREENECT_VIDEO_RGB:
      case FREENECT_VIDEO_YUV_RGB:
        _rgbFrame.copyTo(mat);
        break;
      case FREENECT_VIDEO_IR_8BIT:
        _irFrame8UC1.copyTo(mat);
        break;
      case FREENECT_VIDEO_IR_10BIT:
        _irFrame16UC1.copyTo(mat);
        break;
      default:
        return false;
    }

    _lastCameraFrameSeqNum = _cameraFrameSeqNum;
    return true;
  }

  bool getDepthFrame(cv::Mat& mat) {
    QMutexLocker lock(&_depthFrameMutex);
    if (_lastDepthFrameSeqNum == _depthFrameSeqNum)
      return false;
    _depthFrame.copyTo(mat);
    _lastDepthFrameSeqNum = _depthFrameSeqNum;
    return true;
  }

private:

  uint64_t _cameraFrameSeqNum;
  uint64_t _depthFrameSeqNum;

  uint64_t _lastCameraFrameSeqNum;
  uint64_t _lastDepthFrameSeqNum;

  cv::Mat _depthFrame;
  cv::Mat _rgbFrame;
  cv::Mat _irFrame8UC1;
  cv::Mat _irFrame16UC1;

  QMutex _colorFrameMutex;
  QMutex _depthFrameMutex;

};


KinectV1Sensor::KinectV1Sensor(QObject* parent)
  : QThread(parent)
  , _device(nullptr)
  , _deviceIndex(0)
  , _stop(true)
  , _depthFormat(DEPTH_MM)
  , _imageFormat(RGB)
{
  start();
}

KinectV1Sensor::~KinectV1Sensor() {
  qDebug() << Q_FUNC_INFO;
  stop();
}

void KinectV1Sensor::setDeviceIndex(int deviceIndex) {
  QMutexLocker lock(&_mutex);
  if (deviceIndex != _deviceIndex) {
    stop();
    _deviceIndex = deviceIndex;
    start();
  }
}

void KinectV1Sensor::depthFormat(DepthFormat depthFormat) {
  QMutexLocker lock(&_mutex);
  if (depthFormat != _depthFormat) {
    _depthFormat = depthFormat;
    _device->setDepthFormat((freenect_depth_format)_depthFormat);
  }
}

void KinectV1Sensor::imageFormat(ImageFormat imageFormat) {
  QMutexLocker lock(&_mutex);
  if (imageFormat != _imageFormat) {
    _imageFormat = imageFormat;
    _device->setVideoFormat((freenect_video_format)_imageFormat);
  }
}

void KinectV1Sensor::run()
{
  _stop = false;

  if ((_device = &_freenect.createDevice<FreenectDeviceImpl>(_deviceIndex) )) {
    _device->setVideoFormat((freenect_video_format)_imageFormat);
    _device->setDepthFormat((freenect_depth_format)_depthFormat);
    _device->startVideo();
    _device->startDepth();

    while (!_stop) {
      update();
    }

    _device->stopVideo();
    _device->stopDepth();
    _device = nullptr;
  }
  else {
    qCritical() << "Kinect v1 device" << _deviceIndex << "not found or cannot be opened.";
  }

  _stop = true;
}

void KinectV1Sensor::stop()
{
  _stop = true;
  _waitCondition.wakeAll();
  wait();
}

void KinectV1Sensor::connectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QThread::connectNotify(signal);
}

void KinectV1Sensor::disconnectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QThread::disconnectNotify(signal);
}

void KinectV1Sensor::updateConnectionState() {

  static const QMetaMethod depthSignal = QMetaMethod::fromSignal(&KinectV1Sensor::depth);
  static const QMetaMethod imageSignal = QMetaMethod::fromSignal(&KinectV1Sensor::image);
#if WITH_PCL
  static const QMetaMethod pointCloudSignal = QMetaMethod::fromSignal(&KinectV1Sensor::pointCloud);
#endif

  mUpdateDepthRequired = isSignalConnected(depthSignal);
  mUpdateImageRequired = isSignalConnected(imageSignal);
#if WITH_PCL
  mUpdatePointCloudRequired = isSignalConnected(pointCloudSignal);
#endif

}

void KinectV1Sensor::update() {

  _device->updateState();

  bool haveDepth = false;
  bool haveImage = false;

  if (mUpdateDepthRequired) {
    if ((haveDepth = _device->getDepthFrame(_depthFrame)))
      emit depth(_depthFrame);
  }

  if (mUpdateImageRequired) {
    if ((haveImage = _device->getCameraFrame(_colorFrame)))
      emit image(_colorFrame);
  }

  if (!haveDepth && !haveImage) QThread::sleep((1/30.0));
}
