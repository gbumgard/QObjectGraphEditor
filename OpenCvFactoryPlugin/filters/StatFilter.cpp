#include "StatFilter.h"

#include "OpenCvFactoryPlugin.h"
#include <iostream>
#include <iomanip>

REGISTER_CLASS(StatFilter)

static double depthAdjustedSd(double x) {
  return 1.3973214E-6 * double(x*x) - 0.001299821 * (double)x + 0.6893571;
}

StatFilter::StatFilter(QObject *parent)
  : QObject(parent)
  , _size(0,0)
  , _enable(true)
  , _count(nullptr)
{
}

StatFilter::~StatFilter() {
  delete _count;
}

void StatFilter::clear() {
  _size = cv::Size(0,0);
}

void StatFilter::in(const cv::Mat& mat) {

  if (!_enable || mat.empty()) return;

  cv::Mat sample;
  mat.convertTo(sample,CV_32F);

  if (sample.size() != _size) {
    _size = sample.size();
    //_count = cv::Mat(_size,CV_);
    delete _count;
    _count = new uint32_t[_size.height * _size.width];
    std::memset(_count, 0, _size.height * _size.width * sizeof(uint32_t));
    _aggregateMean = cv::Mat(_size,CV_32F);
    sample.copyTo(_aggregateMean);
    _aggregateMeanSquared = cv::Mat(_size,CV_32F,cv::Scalar(0.0));
    //_aggregateMeanSquared = 0.0;
    _variance = cv::Mat(_size,CV_32F,cv::Scalar(0.0));
    //_variance = 0.0;
    _filtered = cv::Mat(_size,CV_32F,cv::Scalar(0.0));
    //_filtered = 0.0;
  }

  for (int row = 0; row < sample.rows; row++) {

    float* sampleRow = sample.ptr<float>(row);
    //int16_t* countRow = _count.ptr<int16_t>(row);
    uint32_t* countRow = &_count[row * _size.width];
    float* aggregateMeanRow = _aggregateMean.ptr<float>(row);
    float* aggregateMeanSquaredRow = _aggregateMeanSquared.ptr<float>(row);
    float* varRow = _variance.ptr<float>(row);
    float* filteredRow = _filtered.ptr<float>(row);

    for (int col = 0; col < sample.cols; col++) {

      double newSample = sampleRow[col];
      if (newSample == 0) {
        continue;
      }

      uint32_t count = countRow[col] + 1;
      countRow[col] = count;

      double delta1 = newSample - aggregateMeanRow[col];
      aggregateMeanRow[col] += delta1/(double)count;

      double delta2 = newSample - aggregateMeanRow[col];
      double sampleVariance = delta1 * delta2;
      aggregateMeanSquaredRow[col] += sampleVariance;

      if (count > 1) {
        double var = varRow[col];
        varRow[col] = aggregateMeanSquaredRow[col]/(double)(count - 1);
        if (fabs(varRow[col]-var) > 1) {
          //countRow[col] = 0;
          filteredRow[col] = aggregateMeanRow[col] = newSample;
          aggregateMeanSquaredRow[col] = 0.0;
          //varRow[col] = 0.0;
        }
      }
      if (fabs(aggregateMeanRow[col]-filteredRow[col]) > .1) {
        filteredRow[col] = aggregateMeanRow[col];
      }
    }
  }

  int sampleRow = 240, sampleCol = 320;

  std::cout << std::setprecision(4)
            << "count=" << std::setw(6) << _count[sampleRow*_size.width + sampleCol]
            << " sample=" << std::setw(8) << sample.at<float>(sampleRow,sampleCol)
            << " agg-mean=" << std::setw(8) << _aggregateMean.at<float>(sampleRow, sampleCol)
            << " agg-mean-squared=" << std::setw(8) << _aggregateMeanSquared.at<float>(sampleRow, sampleCol)
            << " variance=" << std::setw(8) << _variance.at<float>(sampleRow,sampleCol)
            << " sd=" << std::setw(8) << sqrt(_variance.at<float>(sampleRow,sampleCol))
            << " adj-sd=" << std::setw(8) << depthAdjustedSd(sample.at<float>(sampleRow,sampleCol))
            << std::endl << std::flush;

  emit mean(_aggregateMean);
  emit variance(_variance);

}
