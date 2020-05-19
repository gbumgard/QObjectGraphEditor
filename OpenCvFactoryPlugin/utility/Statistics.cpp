#include "Statistics.h"

#include "OpenCvFactoryPlugin.h"
#include <iostream>
#include <iomanip>

REGISTER_CLASS(Statistics)

Statistics::Statistics(QObject *parent)
  : QObject(parent)
  , _size(0,0)
  , _enable(true)
  , _count(0)
{
}

void Statistics::clear() {
  _count = 0;
  _aggregateMean = 0;
  _aggregateMeanSquared = 0.0;
}

void Statistics::in(const cv::Mat& mat) {

  if (!_enable) return;

  if (!mat.empty()) {
    if (mat.size() != _size) {
      _size = mat.size();
      _aggregateMean = cv::Mat(_size,CV_32F);
      _aggregateMeanSquared = cv::Mat(_size,CV_32F);
      clear();
    }
  }

  /*
   *
   *      (count, mean, M2) = existingAggregate
    count = count + 1
    delta = newValue - mean
    mean = mean + delta / count
    delta2 = newValue - mean
    M2 = M2 + delta * delta2
    */

  _count++;

  cv::Mat sample;
  mat.convertTo(sample,CV_32F);

  cv::Mat delta(_size,CV_32F);
  delta = sample - _aggregateMean;

  _aggregateMean += (delta/_count);

  cv::Mat delta2(_size,CV_32F);
  delta2 = sample - _aggregateMean;

  delta.mul(delta2);
  _aggregateMeanSquared += delta;

  cv::Mat var(_size,CV_32F);
  var = _aggregateMeanSquared / (float)(_count - 1);

  if (_count > 1) {
    emit mean(_aggregateMean);
    emit variance(var);
  }
}
