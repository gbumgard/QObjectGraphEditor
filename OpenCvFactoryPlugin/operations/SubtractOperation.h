#ifndef SUBTRACTOPERATION_H
#define SUBTRACTOPERATION_H

#include <QObject>

#include <opencv2/core.hpp>

class SubtractOperation : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Subtract Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

  Q_PROPERTY(int outputType READ outputType WRITE outputType)

public:

  Q_INVOKABLE explicit SubtractOperation(QObject* parent = nullptr);

  virtual ~SubtractOperation() {}

  int outputType() const { return _type; }
  void outputType(int outputType);

public slots:

  void alpha(const cv::Mat& alpha);
  void beta(const cv::Mat& beta);
  void mask(const cv::Mat& mask);

signals:

  void output(const cv::Mat& mat);

protected:

  void update();

private:

  int _type;

  cv::Mat _alphaMat;
  cv::Mat _betaMat;
  cv::Mat _maskMat;

};

#endif // SUBTRACTOPERATION_H
