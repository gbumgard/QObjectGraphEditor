#include "StatFilter.h"

#include "ObjectModel.h"
#include <iostream>
#include <iomanip>

REGISTER_CLASS(StatFilter)

static double depthAdjustedSd(double x) {
  return 1.3973214E-6 * double(x*x) - 0.001299821 * (double)x + 0.6893571;
}

StatFilter::StatFilter(QObject *parent)
  : AbstractOpenCvObject(parent)
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

void StatFilter::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);

      if (!_enable || matEvent.mat().empty()) return;

      cv::Mat sample;
      matEvent.mat().convertTo(sample,CV_64F);

      if (sample.size() != _size) {
          _size = sample.size();
          //_count = cv::Mat(_size,CV_);
          delete _count;
          _count = new uint32_t[_size.height * _size.width];
          std::memset(_count, 0, _size.height * _size.width * sizeof(uint32_t));
          _aggregateMean = cv::Mat(_size,CV_64F);
          sample.copyTo(_aggregateMean);
          _aggregateMeanSquared = cv::Mat(_size,CV_64F,cv::Scalar(0.0));
          //_aggregateMeanSquared = 0.0;
          _variance = cv::Mat(_size,CV_64F,cv::Scalar(0.0));
          //_variance = 0.0;
          _filtered = cv::Mat(_size,CV_64F,cv::Scalar(0.0));
          //_filtered = 0.0;
      }

      for (int row = 0; row < sample.rows; row++) {

          double* sampleRow = sample.ptr<double>(row);
          //int16_t* countRow = _count.ptr<int16_t>(row);
          uint32_t* countRow = &_count[row * _size.width];
          double* aggregateMeanRow = _aggregateMean.ptr<double>(row);
          double* aggregateMeanSquaredRow = _aggregateMeanSquared.ptr<double>(row);
          double* varRow = _variance.ptr<double>(row);
          double* filteredRow = _filtered.ptr<double>(row);

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

      double minVal, maxVal;
      cv::minMaxLoc(_variance,&minVal,&maxVal);

      int sampleRow = 240, sampleCol = 320;

      std::cout << std::setprecision(4)
                << "count=" << std::setw(6) << _count[sampleRow*_size.width + sampleCol]
                << " sample=" << std::setw(8) << sample.at<double>(sampleRow,sampleCol)
                << " agg-mean=" << std::setw(8) << _aggregateMean.at<double>(sampleRow, sampleCol)
                << " agg-mean-squared=" << std::setw(8) << _aggregateMeanSquared.at<double>(sampleRow, sampleCol)
                << " variance=" << std::setw(8) << _variance.at<double>(sampleRow,sampleCol)
                << " sd=" << std::setw(8) << sqrt(_variance.at<double>(sampleRow,sampleCol))
                << " adj-sd=" << std::setw(8) << depthAdjustedSd(sample.at<double>(sampleRow,sampleCol))
                << " variance-range=" << minVal << "," << maxVal
                << std::endl << std::flush;

      emit mean(QVariant::fromValue(MatEvent(_aggregateMean,matEvent.timestamp())));
      emit variance(QVariant::fromValue(MatEvent(_variance,matEvent.timestamp())));
      emit filtered(QVariant::fromValue(MatEvent(_filtered,matEvent.timestamp())));
  }
  else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
