#ifndef ADDWEIGHTEDOPERATION_H
#define ADDWEIGHTEDOPERATION_H

#include <QObject>
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class AddWeightedOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Add Weighted Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

  Q_PROPERTY(double alphaWeight READ alphaWeight WRITE alphaWeight)
  Q_PROPERTY(double betaWeight READ betaWeight WRITE betaWeight)
  Q_PROPERTY(double gammaOffset READ gammaOffset WRITE gammaOffset)

public:

  Q_INVOKABLE explicit AddWeightedOperation(QObject* parent = nullptr);

  virtual ~AddWeightedOperation() {}

  double alphaWeight() const { return _alphaWeight; }
  double betaWeight() const { return _betaWeight; }
  double gammaOffset() const { return _gammaOffset; }

public slots:

  void alpha(const cv::Mat& alpha);
  void beta(const cv::Mat& beta);

  void alphaWeight(double alphaWeight);
  void betaWeight(double betaWeight);
  void gammaOffset(double gammaOffset);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  double _alphaWeight;
  double _betaWeight;
  double _gammaOffset;

  cv::Mat _alphaMat;
  cv::Mat _betaMat;

};


#endif // ADDWEIGHTEDOPERATION_H
