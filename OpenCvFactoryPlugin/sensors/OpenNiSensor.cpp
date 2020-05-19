#include "OpenCvFactoryPlugin.h"
#include "OpenNiSensor.h"

#include <QMutexLocker>
#include <QMetaObject>
#include <QMetaMethod>
#include <QDebug>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

REGISTER_CLASS(OpenNiSensor)

OpenNiSensor::OpenNiSensor(QObject* parent)
  : QThread(parent)
  , mDeviceIndex(cv::CAP_OPENNI)
  , mKinect()
  , mStop(false)
  , mDepthGeneratorPresent(false)
  , mIrImageGeneratorPresent(false)
  , mColorImageGeneratorPresent(false)
  , mUpdateDepthMapRequired(false)
  , mUpdateDepthMaskRequired(false)
  , mUpdatePointCloudMapRequired(false)
  , mUpdateDisparityMapRequired(false)
  , mUpdateDisparityMap32FRequired(false)
  , mUpdateIrCameraImageRequired(false)
  , mUpdateBGRCameraImageRequired(false)
  , mUpdateGrayCameraImageRequired(false)
{
  open(mDeviceIndex);
  start();
}

OpenNiSensor::~OpenNiSensor()
{
  qDebug() << Q_FUNC_INFO;
  stop();
  close();
}

void OpenNiSensor::deviceIndex(int index) {
  if (index != mDeviceIndex) {
    stop();
    open(index);
  }
}

void OpenNiSensor::stop()
{
  mStop = true;
  mWaitCondition.wakeAll();
  wait();
}

void OpenNiSensor::run()
{
  mMutex.lock();
  if (!mKinect.isOpened()) {
    mWaitCondition.wait(&mMutex);
  }
  mMutex.unlock();

  while (!mStop) {
    updateImages();
  }
}

bool OpenNiSensor::open(int device)
{
  QMutexLocker lock(&mMutex);
  if (mKinect.open(device)) {
    mWaitCondition.wakeAll();

    if (mKinect.get(cv::CAP_OPENNI_DEPTH_GENERATOR_PRESENT)) {
      mDepthGeneratorPresent = true;
      qDebug() << "Depth generator output mode:";
      qDebug() << "FRAME_WIDTH      " << mKinect.get(cv::CAP_PROP_FRAME_WIDTH);
      qDebug() << "FRAME_HEIGHT     " << mKinect.get(cv::CAP_PROP_FRAME_HEIGHT);
      qDebug() << "FRAME_MAX_DEPTH  " << mKinect.get(cv::CAP_PROP_OPENNI_FRAME_MAX_DEPTH);
      qDebug() << "FPS              " << mKinect.get(cv::CAP_PROP_FPS);
      qDebug() << "REGISTRATION     " << mKinect.get(cv::CAP_PROP_OPENNI_REGISTRATION);
    }
    else {
      qDebug() << "Device doesn't contain depth generator or it is not selected.";
    }

    if (mKinect.get(cv::CAP_OPENNI_IMAGE_GENERATOR_PRESENT)) {
      mColorImageGeneratorPresent = true;
      qDebug() << "Image generator output mode:";
      qDebug() << "FRAME_WIDTH      " << mKinect.get(cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_WIDTH);
      qDebug() << "FRAME_HEIGHT     " << mKinect.get(cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT);
      qDebug() << "FPS              " << mKinect.get(cv::CAP_OPENNI_IMAGE_GENERATOR + cv::CAP_PROP_FPS);
    }
    else {
      qDebug() << "\nDevice doesn't contain image generator or it is not selected.";
    }

    if (mKinect.get(cv::CAP_OPENNI_IR_GENERATOR_PRESENT))
                    {
      mIrImageGeneratorPresent = true;
      qDebug() << "IR generator output mode:";
      qDebug() << "FRAME_WIDTH      " << mKinect.get(cv::CAP_OPENNI_IR_GENERATOR + cv::CAP_PROP_FRAME_WIDTH);
      qDebug() << "FRAME_HEIGHT     " << mKinect.get(cv::CAP_OPENNI_IR_GENERATOR + cv::CAP_PROP_FRAME_HEIGHT);
      qDebug() << "FPS              " << mKinect.get(cv::CAP_OPENNI_IR_GENERATOR + cv::CAP_PROP_FPS);
    }
    else {
      qDebug() << "\nDevice doesn't contain IR generator or it is not selected.";
    }
    mDeviceIndex = device;
    return true;
  }
  else {
    mStop = true;
    mWaitCondition.wakeAll();
  }
  return false;
}

void OpenNiSensor::close()
{
  mKinect.release();
}

cv::Mat OpenNiSensor::currentDepthMap() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mDepthMap;
  (void) lock;
}

cv::Mat OpenNiSensor::currentValidDepthMask() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mValidDepthMask;
  (void) lock;
}

cv::Mat OpenNiSensor::currentPointCloudMap() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mPointCloudMap;
  (void) lock;
}

cv::Mat OpenNiSensor::currentDisparityMap() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mDisparityMap;
  (void) lock;
}

cv::Mat OpenNiSensor::currentDisparityMap32F() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mDisparityMap32F;
  (void) lock;
}

cv::Mat OpenNiSensor::currentIrCameraImage() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mIrCameraImage;
  (void) lock;
}

cv::Mat OpenNiSensor::currentBgrCameraImage() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mBGRCameraImage;
  (void) lock;
}

cv::Mat OpenNiSensor::currentGrayCameraImage() const
{
  QMutexLocker lock(const_cast<QMutex*>(&mMutex));
  return mGrayCameraImage;
  (void) lock;
}

void OpenNiSensor::connectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QThread::connectNotify(signal);
}

void OpenNiSensor::disconnectNotify(const QMetaMethod &signal) {
  updateConnectionState();
  QThread::disconnectNotify(signal);
}

void OpenNiSensor::updateConnectionState() {
  /*
  static const QMetaMethod depthMapReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::depthMapReady);
  static const QMetaMethod depthMaskReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::validDepthMaskReady);
  static const QMetaMethod pointCloudMapReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::pointCloudMapReady);
  static const QMetaMethod disparityMapReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::disparityMapReady);
  static const QMetaMethod disparityMap32FReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::disparityMap32FReady);
  static const QMetaMethod irCameraImageReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::irCameraImageReady);
  static const QMetaMethod bgrCameraImageReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::bgrCameraImageReady);
  static const QMetaMethod grayCameraImageReadySignal = QMetaMethod::fromSignal(&OpenNiSensor::grayCameraImageReady);
  */
  static const QMetaMethod depthMapSignal = QMetaMethod::fromSignal(&OpenNiSensor::depthMap);
  static const QMetaMethod depthMaskSignal = QMetaMethod::fromSignal(&OpenNiSensor::validDepthMask);
  static const QMetaMethod pointCloudMapSignal = QMetaMethod::fromSignal(&OpenNiSensor::pointCloudMap);
  static const QMetaMethod disparityMapSignal = QMetaMethod::fromSignal(&OpenNiSensor::disparityMap);
  static const QMetaMethod disparityMap32FSignal = QMetaMethod::fromSignal(&OpenNiSensor::disparityMap32F);
  static const QMetaMethod irCameraImageSignal = QMetaMethod::fromSignal(&OpenNiSensor::irCameraImage);
  static const QMetaMethod bgrCameraImageSignal = QMetaMethod::fromSignal(&OpenNiSensor::bgrCameraImage);
  static const QMetaMethod grayCameraImageSignal = QMetaMethod::fromSignal(&OpenNiSensor::grayCameraImage);
  /*
  mUpdateDepthMapRequired = mDepthGeneratorPresent && (isSignalConnected(depthMapReadySignal) || isSignalConnected(depthMapSignal));
  mUpdateDepthMaskRequired = mDepthGeneratorPresent && (isSignalConnected(depthMaskReadySignal) || isSignalConnected(depthMaskSignal));
  mUpdatePointCloudMapRequired = mDepthGeneratorPresent && (isSignalConnected(pointCloudMapReadySignal) || isSignalConnected(pointCloudMapSignal));
  mUpdateDisparityMapRequired = mDepthGeneratorPresent && (isSignalConnected(disparityMapReadySignal) || isSignalConnected(disparityMapSignal));
  mUpdateDisparityMap32FRequired = mDepthGeneratorPresent && (isSignalConnected(disparityMap32FReadySignal) || isSignalConnected(disparityMap32FSignal));
  mUpdateIrCameraImageRequired = mIrImageGeneratorPresent && (isSignalConnected(irCameraImageReadySignal) || isSignalConnected(irCameraImageSignal));
  mUpdateBGRCameraImageRequired = mColorImageGeneratorPresent && (isSignalConnected(bgrCameraImageReadySignal) || isSignalConnected(bgrCameraImageSignal));
  mUpdateGrayCameraImageRequired = mColorImageGeneratorPresent && (isSignalConnected(grayCameraImageReadySignal) || isSignalConnected(grayCameraImageSignal));
  */
  mUpdateDepthMapRequired = mDepthGeneratorPresent && (isSignalConnected(depthMapSignal));
  mUpdateDepthMaskRequired = mDepthGeneratorPresent && (isSignalConnected(depthMaskSignal));
  mUpdatePointCloudMapRequired = mDepthGeneratorPresent && (isSignalConnected(pointCloudMapSignal));
  mUpdateDisparityMapRequired = mDepthGeneratorPresent && (isSignalConnected(disparityMapSignal));
  mUpdateDisparityMap32FRequired = mDepthGeneratorPresent && (isSignalConnected(disparityMap32FSignal));
  mUpdateIrCameraImageRequired = mIrImageGeneratorPresent && (isSignalConnected(irCameraImageSignal));
  mUpdateBGRCameraImageRequired = mColorImageGeneratorPresent && (isSignalConnected(bgrCameraImageSignal));
  mUpdateGrayCameraImageRequired = mColorImageGeneratorPresent && (isSignalConnected(grayCameraImageSignal));
  qDebug() << Q_FUNC_INFO
           << mUpdateDepthMapRequired
           << mUpdateDepthMaskRequired
           << mUpdatePointCloudMapRequired
           << mUpdateDisparityMapRequired
           << mUpdateDisparityMap32FRequired
           << mUpdateIrCameraImageRequired
           << mUpdateBGRCameraImageRequired
           << mUpdateGrayCameraImageRequired;
}

void OpenNiSensor::updateImages()
{
  if (mUpdateDepthMapRequired ||
      mUpdateDepthMaskRequired ||
      mUpdatePointCloudMapRequired ||
      mUpdateDisparityMapRequired ||
      mUpdateDisparityMap32FRequired ||
      mUpdateIrCameraImageRequired ||
      mUpdateBGRCameraImageRequired ||
      mUpdateGrayCameraImageRequired) {
    if (mKinect.grab()) {
      if (mUpdateDepthMapRequired) updateDepthMap();
      if (mUpdateDepthMaskRequired) updateValidDepthMask();
      if (mUpdatePointCloudMapRequired) updatePointCloudMap();
      if (mUpdateDisparityMapRequired) updateDisparityMap();
      if (mUpdateDisparityMap32FRequired) updateDisparityMap();
      if (mUpdateIrCameraImageRequired) updateIrCameraImage();
      if (mUpdateBGRCameraImageRequired) updateBGRCameraImage();
      if (mUpdateGrayCameraImageRequired) updateGrayCameraImage();
    }
  }
}

void OpenNiSensor::updateDepthMap()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_DEPTH_MAP)) {
    QMutexLocker lock(&mMutex);
    mDepthMap = image;
    emit depthMap(mDepthMap);
    (void) lock;
  }
}

void OpenNiSensor::updateValidDepthMask()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_VALID_DEPTH_MASK)) {
    QMutexLocker lock(&mMutex);
    mValidDepthMask = image;
    emit validDepthMask(mValidDepthMask);
    (void) lock;
  }
}

void OpenNiSensor::updatePointCloudMap()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_POINT_CLOUD_MAP)) {
    QMutexLocker lock(&mMutex);
    mPointCloudMap = image;
    emit pointCloudMap(mPointCloudMap);
    (void) lock;
  }
}

void OpenNiSensor::updateDisparityMap()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_DISPARITY_MAP)) {
    QMutexLocker lock(&mMutex);
    mDisparityMap = image;
    emit disparityMap(mDisparityMap);
    (void) lock;
  }
}

void OpenNiSensor::updateDisparityMap32F()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_DISPARITY_MAP_32F)) {
    QMutexLocker lock(&mMutex);
    mDisparityMap32F = image;
    emit disparityMap32F(mDisparityMap32F);
    (void) lock;
  }
}

void OpenNiSensor::updateIrCameraImage()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_IR_IMAGE)) {
    QMutexLocker lock(&mMutex);
    mIrCameraImage = image;
    //emit irCameraImageReady();
    emit irCameraImage(mIrCameraImage);
    (void) lock;
  }
}

void OpenNiSensor::updateBGRCameraImage()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_BGR_IMAGE)) {
    QMutexLocker lock(&mMutex);
    mBGRCameraImage = image;
    emit bgrCameraImage(mBGRCameraImage);
    (void) lock;
  }
}

void OpenNiSensor::updateGrayCameraImage()
{
  cv::Mat image;
  if (mKinect.retrieve(image, cv::CAP_OPENNI_GRAY_IMAGE)) {
    QMutexLocker lock(&mMutex);
    mGrayCameraImage = image;
    emit grayCameraImage(mGrayCameraImage);
    (void) lock;
  }
}

/*
void OpenNiSensor::emitDepthMap() {
  emit depthMap(currentDepthMap());
}

void OpenNiSensor::emitValidDepthMask() {
  emit validDepthMask(currentValidDepthMask());
}

void OpenNiSensor::emitPointCloudMap() {
  emit pointCloudMap(currentPointCloudMap());
}

void OpenNiSensor::emitDisparityMap() {
  emit disparityMap(currentDisparityMap());
}

void OpenNiSensor::emitDisparityMap32F() {
  emit disparityMap32F(currentDisparityMap32F());
}

void OpenNiSensor::emitIrCameraImage() {
  emit irCameraImage(currentIrCameraImage());
}

void OpenNiSensor::emitBgrCameraImage() {
  emit bgrCameraImage(currentBgrCameraImage());
}

void OpenNiSensor::emitGrayCameraImage() {
  emit grayCameraImage(currentGrayCameraImage());
}
*/
