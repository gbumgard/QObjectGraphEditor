#include "filters/MedianFilter3D.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <future>
#include <QThreadPool>

REGISTER_CLASS(MedianFilter3D)

MedianFilter3D::MedianFilter3D(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _aperatureSize(Kernel3x3x3)
  , _taskFuture()
{
    //std::promise<void> promise;
    //_taskFuture = promise.get_future();
    //promise.set_value();
    QObject::connect(this,&MedianFilter3D::finished,this,&MedianFilter3D::start,Qt::ConnectionType::AutoConnection);
}

void MedianFilter3D::setAperatureSize(AperatureSize aperatureSize) {
  _aperatureSize = aperatureSize;
}

void MedianFilter3D::in(const QVariant &variant) {

  if (variant.userType() == MatEvent::userType()) {

      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);

      cv::Mat mat(matEvent.mat());
      /*
      switch (matEvent.mat().depth()) {
      case CV_8U:
      case CV_16U:
      case CV_32F:
          break;
      default:
          matEvent.mat().convertTo(mat,CV_32F);
          break;
      }
      */

      if (!_frameBuffer.empty() && (mat.size() != _frameBuffer.front().size() || mat.type() != _frameBuffer.front().type())) {
          qDebug() << Q_FUNC_INFO << "init buffer for size change";
          _frameBuffer.clear();
      }

      while (_frameBuffer.size() > (size_t)_aperatureSize - 1) {
          _frameBuffer.erase(_frameBuffer.begin());
      }

      //cv::Mat output(mat.rows,mat.cols,mat.type(),cv::Scalar(0));
      //try {
      //    cv::medianBlur(mat,output,(int)_aperatureSize);
      //}
      //catch(cv::Exception &e) {
      //    qDebug() << e.what();
      //}
      //_frameBuffer.push_back(output);

      _frameBuffer.push_back(mat.clone());

      if (_frameBuffer.size() < _aperatureSize) {

          while (_frameBuffer.size() < _aperatureSize) {
              _frameBuffer.push_back(mat.clone());
          }

          emit finished();
      }
    }
    else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported input type");
    }
}

void MedianFilter3D::start() {

  QThreadPool::globalInstance()->start([&]() {
          auto frameBuffer(_frameBuffer);

          cv::Mat output(frameBuffer[0].rows,frameBuffer[0].cols,frameBuffer[0].type());

          switch (frameBuffer[0].depth()) {
          case CV_8U: {
              computeMedian<uint8_t>(frameBuffer,output);
          }
          break;
          case CV_8S: {
              computeMedian<int8_t>(frameBuffer,output);
          }
          break;
          case CV_16U: {
              computeMedian<uint16_t>(frameBuffer,output);
          }
          break;
          case CV_16S: {
              computeMedian<int16_t>(frameBuffer,output);
          }
          break;
          case CV_32S: {
              computeMedian<int32_t>(frameBuffer,output);
          }
          break;
          case CV_32F: {
              computeMedian<float>(frameBuffer,output);
          }
          break;
          case CV_64F: {
              computeMedian<double>(frameBuffer,output);
          }
          break;
          }
          emit out(QVariant::fromValue(MatEvent(output)));
          emit finished();
    });
}

