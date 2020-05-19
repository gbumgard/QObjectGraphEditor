#include "MatToQImageConvertor.h"
#include "OpenCvFactoryPlugin.h"
#include <QImage>

REGISTER_CLASS(MatToQImageConvertor)

MatToQImageConvertor::MatToQImageConvertor(QObject* parent)
  : AbstractOpenCvObject(parent)
{

}

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

    //qDebug() << "mat type=" << mat.type();
    switch (mat.type()) {
      case CV_8UC3:
        //qDebug() << " CV_8UC3";
        if (swap) {
          return mat_to_qimage_ref(mat, QImage::Format_RGB888).rgbSwapped();
        }
        else {
          return mat_to_qimage_ref(mat, QImage::Format_RGB888);
        }
      case CV_8UC1:
        //qDebug() << " CV_8UC1";
        return mat_to_qimage_ref(mat, QImage::Format_Grayscale8);

      case CV_8UC4:
        //qDebug() << " CV_8UC4";
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

void MatToQImageConvertor::convert(const cv::Mat &mat) {
  emit conversion(mat_to_qimage_cpy(mat,true));
}
