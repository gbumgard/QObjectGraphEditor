#include "DepthFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <math.h>
#include <iostream>
#include <iomanip>

REGISTER_CLASS(DepthFilter)

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

DepthFilter::DepthFilter(QObject *parent)
  : QObject(parent)
  , _size(0,0)
  , _fastGain(.9)
  , _slowGain(.1)
  , _hysteresis(0)
{
}

void DepthFilter::fastGain(qreal gain) {
  if (gain > 0 && gain <= 1.0) {
    _fastGain = gain;
  }
}

void DepthFilter::slowGain(qreal gain) {
  if (gain > 0.0 && gain < _fastGain) {
    _slowGain = gain;
  }
}

void DepthFilter::hysteresis(qreal level) {
  if (level >= 0.0) {
    _hysteresis = level;
  }
}

void DepthFilter::in(const cv::Mat &mat) {

  if (mat.empty()) return;

  cv::Mat sample;
  if (mat.type() != CV_32F)
    mat.convertTo(sample,CV_32F);
  else
    sample = mat;

  // First sample is simply queued and emitted.
  if (sample.size() != _size) {

    _size = sample.size();

    _fastAverage = cv::Mat(_size,CV_64F);
    _slowAverage = cv::Mat(_size,CV_64F);
    _slowVariance = cv::Mat(_size,CV_64F,cv::Scalar(0.0));
    _filteredSample = cv::Mat(_size,CV_32F);

    sample.convertTo(_fastAverage,CV_64F);
    sample.convertTo(_slowAverage,CV_64F);

    emit out(sample);

    return;
  }

  double varianceGain = _fastGain;

  for (int row = 0; row < sample.rows; row++) {

    float* sampleRow = sample.ptr<float>(row);
    double* fastAverageRow = _fastAverage.ptr<double>(row);
    double* slowAverageRow = _slowAverage.ptr<double>(row);
    double* slowVarianceRow = _slowVariance.ptr<double>(row);
    float* filteredRow = _filteredSample.ptr<float>(row);

    for (int col = 0; col < sample.cols; col++) {

      double sample = sampleRow[col];
      if (sample == 0) {
        continue;
      }

      fastAverageRow[col] = (1.0 - _fastGain) * fastAverageRow[col] + _fastGain * sample;

      double slowDelta1 = sample - slowAverageRow[col];
      slowAverageRow[col] = (1.0 - _slowGain) * slowAverageRow[col] + _slowGain * sample;
      double slowDelta2 = sample - slowAverageRow[col];

      double slowVariance = slowDelta1 * slowDelta2;
      slowVarianceRow[col] = (1.0 - varianceGain) * slowVarianceRow[col] + varianceGain * slowVariance;

      float filtered;
      double averageDelta = fabs(fastAverageRow[col] - slowAverageRow[col]);

      if (averageDelta*averageDelta > slowVarianceRow[col]) {
        filtered = slowAverageRow[col] = fastAverageRow[col];
        slowVarianceRow[col] = 0;
      }
      else {
        filtered = slowAverageRow[col];
      }

      if (fabs(filtered - filteredRow[col]) > _hysteresis)
        filteredRow[col] = filtered;
    }
  }

  int sampleRow = 240, sampleCol = 320;

  std::cout << std::setprecision(6)
            << " sample=" << std::setw(8) << sample.at<float>(sampleRow,sampleCol)
            << " filtered=" << std::setw(8) << _filteredSample.at<float>(sampleRow,sampleCol)
            << " fast-mean=" << std::setw(8) << _fastAverage.at<double>(sampleRow, sampleCol)
            << " slow-mean=" << std::setw(8) << _slowAverage.at<double>(sampleRow, sampleCol)
            << " variance-mean=" << std::setw(10) << _slowVariance.at<double>(sampleRow, sampleCol)
            << std::endl << std::flush;

  emit out(_filteredSample);
}
