#ifndef ADDOPERATION_H
#define ADDOPERATION_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class AddOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Add Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

  Q_PROPERTY(int outputType READ outputType WRITE setOutputType)

public:

  Q_INVOKABLE explicit AddOperation(QObject* parent = nullptr);

  virtual ~AddOperation() {}

  int outputType() const { return _type; }

public slots:

  void alpha(const cv::Mat& alpha);
  void beta(const cv::Mat& beta);
  void mask(const cv::Mat& mask);

  void setOutputType(int outputType);

signals:

  void out(const cv::Mat& mat);
  void maskUsed(const cv::Mat& mat);

protected:

  void update();

private:

  double _type;

  cv::Mat _alphaMat;
  cv::Mat _betaMat;
  cv::Mat _maskMat;

};

#endif // ADDOPERATION_H
