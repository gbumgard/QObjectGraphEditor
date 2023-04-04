#ifndef ADDWEIGHTEDOPERATION_H
#define ADDWEIGHTEDOPERATION_H

#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <opencv2/core.hpp>

class AddWeightedOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Add Weighted Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

  Q_PROPERTY(double alphaWeight READ alphaWeight WRITE alphaWeight NOTIFY alphaWeightChanged)
  Q_PROPERTY(double betaWeight READ betaWeight WRITE betaWeight NOTIFY betaWeightChanged)
  Q_PROPERTY(double gammaOffset READ gammaOffset WRITE gammaOffset NOTIFY gammaOffsetChanged)

public:

  Q_INVOKABLE explicit AddWeightedOperation(QObject* parent = nullptr);

  virtual ~AddWeightedOperation() {}

  double alphaWeight() const { return _alphaWeight; }
  double betaWeight() const { return _betaWeight; }
  double gammaOffset() const { return _gammaOffset; }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void alpha(const QVariant& srcEvent);

  QVARIANT_PAYLOAD(MatEvent) void beta(const QVariant& srcEvent);

  void alphaWeight(double alphaWeight);
  void betaWeight(double betaWeight);
  void gammaOffset(double gammaOffset);

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void alphaWeightChanged(double);
  void betaWeightChanged(double);
  void gammaOffsetChanged(double);

protected:

  void update();

private:

  double _alphaWeight;
  double _betaWeight;
  double _gammaOffset;

  cv::Mat _alphaMat;
  cv::Mat _betaMat;
  int64_t _timestamp;

};


#endif // ADDWEIGHTEDOPERATION_H
