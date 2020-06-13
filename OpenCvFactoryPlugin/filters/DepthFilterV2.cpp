#include "DepthFilterV2.h"
#include "OpenCvFactoryPlugin.h"

#include <math.h>
#include <iostream>
#include <iomanip>
#include <opencv2/imgproc.hpp>

REGISTER_CLASS(DepthFilterV2)

/*
static double* initSDTable() {
  static double table[0x1FFF];
  for (int x = 0; x < 0x1FFF; x++) {
    table[x] = 1.666666667E-5 * double(x*x) - 0.00771428571 * (double)x + 2.83333333;
    qDebug() << x << table[x];
  }
  return table;
}

static const double* getKinectV1SDTable() {
  static const double* table = initSDTable();
  return table;
}
*/

DepthFilterV2::DepthFilterV2(QObject *parent)
  : QObject(parent)
  , _range(0)
  , _fastGain(.9)
  , _slowGain(.1)
  , _min(std::numeric_limits<double>::lowest())
  , _max(std::numeric_limits<double>::max())
{
}

void DepthFilterV2::fastGain(qreal gain) {
  if (gain > 0 && gain <= 1.0) {
    _fastGain = gain;
    reset();
  }
}

void DepthFilterV2::slowGain(qreal gain) {
  if (gain > 0.0 && gain <= 1.0) {
    _slowGain = gain;
    reset();
  }
}

void DepthFilterV2::range(qreal range) {
  if (range >= 0.0) {
    _range = range;
  }
}

void DepthFilterV2::reset() {
  _fastAverage = 0.0;
  _slowAverage = 0.0;
  _average = 0.0;
}

void DepthFilterV2::in(const TaggedMat &mat) {

  if (mat.first.empty()) return;

  cv::Mat sample;
  if (mat.first.type() != CV_64F)
    mat.first.convertTo(sample,CV_64F);
  else
    sample = mat.first;

  cv::Mat aboveMinMask = mat.first >= _min;
  cv::Mat belowMaxMask = mat.first <= _max;
  cv::Mat mask = aboveMinMask & belowMaxMask;

  emit msk(TaggedMat(mask,mat.second));

  // First sample is simply queued and emitted.
  if (sample.size() != _average.size()) {

    qDebug() << "initialize depth filter for samples" << sample.cols << sample.rows;

    sample.copyTo(_fastAverage,mask);
    sample.copyTo(_slowAverage,mask);
    sample.copyTo(_average,mask);
    _slowVariance = cv::Mat(sample.size(),CV_64F,cv::Scalar(0.0));
    _fastVariance = cv::Mat(sample.size(),CV_64F,cv::Scalar(0.0));

    qDebug() << "_fastAverage" << _fastAverage.cols << _fastAverage.rows << _fastAverage.type() << _fastAverage.depth();
    qDebug() << "_fastVariance" << _fastVariance.cols << _fastVariance.rows << _fastVariance.type() << _fastVariance.depth();
    qDebug() << "_slowAverage" << _slowAverage.cols << _slowAverage.rows << _slowAverage.type() << _slowAverage.depth();
    qDebug() << "_slowVariance" << _slowVariance.cols << _slowVariance.rows << _slowVariance.type() << _slowVariance.depth();

    emit fast(TaggedMat(_fastAverage,mat.second));
    emit fastVar(TaggedMat(_fastVariance,mat.second));
    emit fastStd(TaggedMat(_fastVariance,mat.second));
    emit slow(TaggedMat(_slowAverage,mat.second));
    emit slowVar(TaggedMat(_slowVariance,mat.second));
    emit slowStd(TaggedMat(_slowVariance,mat.second));
    emit out(TaggedMat(_average,mat.second));

    return;
  }

  cv::accumulateWeighted(sample,_fastAverage,_fastGain,mask);
  cv::accumulateWeighted(sample,_slowAverage,_slowGain,mask);

  cv::Mat slowDelta;
  cv::absdiff(sample,_slowAverage,slowDelta);
  cv::Mat slowVariance = slowDelta;
  slowVariance.mul(slowDelta);
  cv::accumulateWeighted(slowVariance,_slowVariance,_slowGain, mask);

  cv::Mat fastDelta;
  cv::absdiff(sample,_fastAverage,fastDelta);
  cv::Mat fastVariance = fastDelta;
  fastVariance.mul(fastDelta);
  cv::accumulateWeighted(fastVariance,_fastVariance,_fastGain, mask);

#if 0
  cv::Mat diff;
  cv::absdiff(slowDelta,fastDelta,diff);
  cv::Mat alphaMat;
  cv::normalize(diff,alphaMat,1,0,cv::NORM_MINMAX,CV_64F,mask);

  double minAlpha, maxAlpha, minMinus, maxMinus;
  cv::minMaxLoc(alphaMat,&minAlpha,&maxAlpha);
#endif

  cv::Mat fastStdMat;
  cv::sqrt(_fastVariance,fastStdMat);
  cv::Mat slowStdMat;
  cv::sqrt(_slowVariance,slowStdMat);

  cv::Mat alphaMat = slowStdMat;
  alphaMat /= 40.0;
  alphaMat += .01;
  cv::Mat greaterThanOneMask = alphaMat > 1.0;
  alphaMat.setTo(cv::Scalar(1.0),greaterThanOneMask);
  //cv::normalize(_slowVariance,alphaMat,1,0,cv::NORM_MINMAX,CV_64F,mask);

  cv::Mat oneMinusAlpha = 1.0 - alphaMat;
  //cv::minMaxLoc(oneMinusAlpha,&minMinus,&maxMinus);

  cv::Mat average = _average.clone();
  cv::Mat prod1 = average.mul(oneMinusAlpha);
  cv::Mat prod2 = sample.mul(alphaMat);
  cv::Mat update = prod1 + prod2;
  update.copyTo(_average,mask);

/*
  cv::accumulateProduct(_average,oneMinusAlpha,_average,_mask);
  cv::accumulateProduct(sample,alphaMat,_average,_mask);
*/

  /*
  qDebug() << "_fastAverage" << _fastAverage.cols << _fastAverage.rows << _fastAverage.type() << _fastAverage.depth();
  qDebug() << "_fastVariance" << _fastVariance.cols << _fastVariance.rows << _fastVariance.type() << _fastVariance.depth();
  qDebug() << "_slowAverage" << _slowAverage.cols << _slowAverage.rows << _slowAverage.type() << _slowAverage.depth();
  qDebug() << "_slowVariance" << _slowVariance.cols << _slowVariance.rows << _slowVariance.type() << _slowVariance.depth();
  */

  emit out(TaggedMat(_average,mat.second));
  emit fast(TaggedMat(_fastAverage,mat.second));
  emit fastVar(TaggedMat(_fastVariance,mat.second));
  emit fastStd(TaggedMat(fastStdMat,mat.second));
  emit slow(TaggedMat(_slowAverage,mat.second));
  emit slowVar(TaggedMat(_slowVariance,mat.second));
  emit slowStd(TaggedMat(slowStdMat,mat.second));
  emit alpha(TaggedMat(alphaMat,mat.second));

}
