#ifndef MIRRORMASKOPERATION_H
#define MIRRORMASKOPERATION_H

#include <QObject>

#include <opencv2/core.hpp>

class MirrorMaskOperation : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mirror Mask Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

public:

  Q_INVOKABLE explicit MirrorMaskOperation(QObject* parent = nullptr);

  virtual ~MirrorMaskOperation() {}

public slots:

  void mask(const cv::Mat& mat);

signals:

  void original(const cv::Mat& mat);
  void inverse(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _mask;

};

#endif // MIRRORMASKOPERATION_H
