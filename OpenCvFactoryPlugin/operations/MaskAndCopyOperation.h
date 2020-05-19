#ifndef COPYMASKOPERATION_H
#define COPYMASKOPERATION_H

#include <QObject>

#include <opencv2/core.hpp>

class MaskAndCopyOperation : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mask And Copy")
  Q_CLASSINFO("directory","OpenCV/Operations")

public:

  Q_INVOKABLE explicit MaskAndCopyOperation(QObject* parent = nullptr);

  virtual ~MaskAndCopyOperation() {}

public slots:

  void in(const cv::Mat& mat);
  void mask(const cv::Mat& mat);

signals:

  void out(const cv::Mat& mat);
  void maskUsed(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _input;
  cv::Mat _mask;
  cv::Mat _output;

};

#endif // COPYMASKOPERATION_H
