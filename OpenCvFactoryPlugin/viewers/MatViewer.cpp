#include "MatViewer.h"
#include "ObjectModel.h"
#include <QPainter>
#include <QDebug>

REGISTER_CLASS(MatViewer)

inline QImage mat_to_qimage_ref(cv::Mat &mat, QImage::Format format)
{
  return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), format);
}

inline cv::Mat qimage_to_mat_ref(QImage &img, int format)
{
  return cv::Mat(img.height(), img.width(), format, img.bits(), img.bytesPerLine());
}

/**
 *@brief make Qimage and cv::Mat share the same buffer, the resource
 * of the cv::Mat must not deleted before the QImage finish
 * the jobs.
 *
 *@param mat : input mat
 *@param swap : true : swap BGR to RGB; false, do nothing
 */
inline QImage mat_to_qimage_ref(cv::Mat &mat, bool swap)
{
  if (!mat.empty()) {

    switch (mat.type()) {
      case CV_8UC3:
        if (swap) {
          return mat_to_qimage_ref(mat, QImage::Format_RGB888).rgbSwapped();
        }
        else {
          return mat_to_qimage_ref(mat, QImage::Format_RGB888);
        }
      case CV_8UC1:
        return mat_to_qimage_ref(mat, QImage::Format_Grayscale8);

      case CV_8UC4:
        return mat_to_qimage_ref(mat, QImage::Format_ARGB32);
    }
  }

  return {};
}

/**
 *@brief copy cv::Mat into QImage
 *
 *@param mat : input mat
 *@param swap : true : swap BGR to RGB; false, do nothing
 */
inline QImage mat_to_qimage_cpy(cv::Mat const &mat, bool swap)
{
  return mat_to_qimage_ref(const_cast<cv::Mat&>(mat), swap).copy();
}

MatViewer::MatViewer(QObject *parent)
  : QWidget()
  , _swapRedBlue(false)
  , _imageSize()
{
  qDebug() << Q_FUNC_INFO;
  QObject::setParent(parent);
  setProperty("methodOffset",QWidget::staticMetaObject.methodOffset());
  setAutoFillBackground(false);
  setAttribute(Qt::WA_NoSystemBackground, true);
  setSizePolicy(QSizePolicy::MinimumExpanding,QSizePolicy::MinimumExpanding);
}

void MatViewer::in(const QVariant &variant) {

  if (variant.userType() == MatEvent::userType()) {

    MatEvent matEvent(qvariant_cast<MatEvent>(variant));
    cv::Mat mat8UC3;

    matEvent.mat().convertTo(mat8UC3,CV_8UC3);
    _image = mat_to_qimage_cpy(mat8UC3,_swapRedBlue);

    if (matEvent.mat().size() != _imageSize) {
      _imageSize = matEvent.mat().size();
      setMinimumSize(_imageSize.width,_imageSize.height);
      adjustSize();
    }

    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK);

    update();
  }
  else if (variant.userType() == QVariant::fromValue(cv::Mat()).userType()) {

    cv::Mat mat(qvariant_cast<cv::Mat>(variant));
    cv::Mat mat8UC3;

    mat.convertTo(mat8UC3,CV_8UC3);

    _image = mat_to_qimage_cpy(mat8UC3,_swapRedBlue);
    if (mat.size() != _imageSize) {
      _imageSize = mat.size();
      setMinimumSize(_imageSize.width,_imageSize.height);
      adjustSize();
    }

    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK);

    update();
  }
  else if (variant.typeId() == QMetaType::QImage) {

    QImage _image = qvariant_cast<QImage>(variant);

    if (_image.size() != QSize(_imageSize.width,_imageSize.height)) {
      _imageSize = cv::Size(_image.size().width(),_image.size().height());
      setMinimumSize(_imageSize.width,_imageSize.height);
      adjustSize();
    }

    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK);

    update();
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Invalid SRC");
  }
}

MatViewer::~MatViewer() {
  qDebug() << Q_FUNC_INFO;
}

void MatViewer::paintEvent(QPaintEvent* /*event*/)
{
  if (!_image.isNull()) {
    QPainter painter(this);
    painter.drawImage(QPoint(0,0),_image.scaled(this->size(),Qt::KeepAspectRatio));
    painter.end();
  }
}
