#ifndef CANNYLINEDETECTOR_H
#define CANNYLINEDETECTOR_H

#include <QObject>
#include "ThreadedObject.h"
#include <opencv2/imgproc.hpp>

class CannyLineDetector : public ThreadedObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Canny Line Detector")
  Q_CLASSINFO("directory","OpenCV/Detectors")

public:

  enum AperatureSize {
    APERATURE_3X3 = 3,
    APERATURE_5x5 = 5,
    APERATURE_7x7 = 7
  };

  Q_ENUM(AperatureSize)

private:

  Q_PROPERTY(double threshold READ threshold WRITE threshold)
  Q_PROPERTY(double ratio READ ratio WRITE ratio)
  Q_PROPERTY(AperatureSize aperatureSize READ aperatureSize WRITE aperatureSize)
  Q_PROPERTY(bool enableL2Gradient READ enableL2Gradient WRITE enableL2Gradient)

public:

  Q_INVOKABLE explicit CannyLineDetector(QObject* parent = nullptr);

  virtual ~CannyLineDetector();

  double threshold() const { return _threshold; }
  double ratio() const { return _ratio; }
  AperatureSize aperatureSize() const { return _aperatureSize; }
  bool enableL2Gradient() const { return _enableL2Gradient; }

public slots:

  void in(const cv::Mat& mat);

  void threshold(double value);
  void ratio(double value);
  void aperatureSize(AperatureSize aperatureSize);
  void enableL2Gradient(bool value);


signals:

  void out(const cv::Mat& mat);

protected:

  void update() override;

private:

  cv::Mat _nextImage;

  double _threshold;
  double _ratio;
  AperatureSize _aperatureSize;
  bool _enableL2Gradient;

};

#endif // CANNYLINEDETECTOR_H
