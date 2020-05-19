#ifndef MATTOQIMAGECONVERTOR_H
#define MATTOQIMAGECONVERTOR_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MatToQImageConvertor : public AbstractOpenCvObject
{
  Q_OBJECT

  Q_CLASSINFO("converter","true")
  Q_CLASSINFO("class-alias","Mat to QImage")
  Q_CLASSINFO("directory","OpenCV/Converters")

public:

  Q_INVOKABLE MatToQImageConvertor(QObject* parent = nullptr);

signals:

  void conversion(const QImage& image);

public slots:

  void convert(const cv::Mat& mat);

};

#endif // MATTOQIMAGECONVERTOR_H
