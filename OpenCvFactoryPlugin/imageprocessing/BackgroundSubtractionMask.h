#ifndef BACKGROUNDSUBTRACTIONMASK_H
#define BACKGROUNDSUBTRACTIONMASK_H

#include <QObject>
#include <opencv2/bgsegm.hpp>

class BackgroundSubtractionMask : public QObject
{

  Q_OBJECT

public:

  enum Algorithm {
    MOG2,
    KNN
  };

  Q_ENUM(Algorithm)

private:

  Q_CLASSINFO("class-alias","Background Subtraction Mask")
  Q_CLASSINFO("directory","OpenCV/Image Processing")

  Q_PROPERTY(Algorithm Algorithm READ algorithm WRITE algorithm)
  Q_PROPERTY(double Learning_Rate READ learningRate WRITE learningRate)

public:

  Q_INVOKABLE BackgroundSubtractionMask(QObject* parent = nullptr);

  Algorithm algorithm() const { return _algorithm; }
  void algorithm(Algorithm algorithm);

  void learningRate(double learningRate);
  double learningRate() const { return _learningRate; }

signals:

  void mask(const cv::Mat& mat);
  void bgnd(const cv::Mat& mat);

public slots:

  void in(const cv::Mat& mat);

private:

  cv::Ptr<cv::BackgroundSubtractor> _pSubtractor;
  Algorithm _algorithm;
  double _learningRate;

};

#endif // BACKGROUNDSUBTRACTIONMASK_H
