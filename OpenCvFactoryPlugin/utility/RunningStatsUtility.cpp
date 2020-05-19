#include "RunningStatsUtility.h"
#include "OpenCvFactoryPlugin.h"
#include <iostream>
#include <iomanip>

REGISTER_CLASS(RunningStatsUtility)

RunningStatsUtility::RunningStatsUtility(QObject *parent)
  : QObject(parent)
  , _minimumSamples(30)
  , _resetThreshold(1.5)
  , _size(0,0)
{
}

void RunningStatsUtility::clear() {
  if (!_count.empty()) {
    _count = 0;
    _oldRunningMean = 0;
    _newRunningMean = 0.0;
    _oldRunningSigma = 0.0;
    _newRunningSigma = 0.0;
    _variance = 0.0;
    _sigma = 0.0;
  }
}

void RunningStatsUtility::in(const cv::Mat &mat) {
  if (!mat.empty()) {
    if (mat.size() != _size) {
      _size = mat.size();
      _count = cv::Mat(_size,CV_32S);
      _oldRunningMean = cv::Mat(_size,CV_32F);
      _newRunningMean = cv::Mat(_size,CV_32F);
      _oldRunningSigma = cv::Mat(_size,CV_32F);
      _newRunningSigma = cv::Mat(_size,CV_32F);
      _variance = cv::Mat(_size,CV_32F);
      _sigma = cv::Mat(_size,CV_32F);
      clear();
    }
  }

  cv::Mat sample;
  mat.convertTo(sample,CV_32F);

  for(int row = 0; row < mat.rows; row++)
  {
    const float* sampleRow = sample.ptr<float>(row);
    uint32_t* countRow = _count.ptr<uint32_t>(row);
    float* _oldMeanRow = _oldRunningMean.ptr<float>(row);
    float* _newMeanRow = _newRunningMean.ptr<float>(row);
    float* _oldSigmaRow = _oldRunningSigma.ptr<float>(row);
    float* _newSigmaRow = _newRunningSigma.ptr<float>(row);
    float* _varianceRow = _variance.ptr<float>(row);
    float* _sigmaRow = _sigma.ptr<float>(row);
    for(int col = 0; col < mat.cols; col++) {
      countRow[col]++;
      // See Knuth TAOCP vol 2, 3rd edition, page 232
      if (countRow[col] == 1)
      {
        _oldMeanRow[col] = _newMeanRow[col] = sampleRow[col];
        _oldSigmaRow[col] = 0.0;
      }
      else if (countRow[col] > (unsigned)_minimumSamples && fabs(sampleRow[col] - _newMeanRow[col]) > (_sigmaRow[col] * _resetThreshold))
      {
        _oldMeanRow[col] = _newMeanRow[col] = sampleRow[col];
        _oldSigmaRow[col] = 0.0;
        countRow[col] = 1;
      }
      else
      {
        _newMeanRow[col] = _oldMeanRow[col] + (sampleRow[col] - _oldMeanRow[col])/countRow[col];
        _newSigmaRow[col] = _oldSigmaRow[col] + (sampleRow[col] - _oldMeanRow[col])*(sampleRow[col] - _newMeanRow[col]);

        // set up for next iteration
        _oldMeanRow[col] = _newMeanRow[col];
        _oldSigmaRow[col] = _newSigmaRow[col];
      }
      float variance = 0;
      if (countRow[col] > 1) {
        variance = _newSigmaRow[col] / (countRow[col] - 1);
      }
      _varianceRow[col] = variance;
      _sigmaRow[col] = sqrt(variance);
    }
  }
  std::cout << std::showpos << std::showpoint
            << "count=" << std::setw(6) << _count.at<uint32_t>(240,320)
            << " sample=" << std::setw(10) << sample.at<float>(240,320)
            << " v=" << std::setw(10) << _variance.at<float>(240,320)
            << " sd=" << std::setw(10) << _sigma.at<float>(240,320)
            << " oM-delta=" << std::setw(10) << fabs(sample.at<float>(240,320)-_newRunningMean.at<float>(240,320))
            << " nM=" << std::setw(10) << _newRunningMean.at<float>(240,320)
            << " oM=" << std::setw(10) << _oldRunningMean.at<float>(240,320)
            << " nS=" << std::setw(10) << _newRunningSigma.at<float>(240,320)
            << " oS=" << std::setw(10) << _oldRunningSigma.at<float>(240,320)
            << std::endl << std::flush;

#if 0
  qDebug() << sample.at<float>(240,320)
           << fabs(sample.at<float>(240,320)-_oldRunningMean.at<float>(240,320))
           << _oldRunningMean.at<float>(240,320)
           << _newRunningMean.at<float>(240,320)
           << _oldRunningSigma.at<float>(240,320)
           << _newRunningSigma.at<float>(240,320)
           << _variance.at<float>(240,320)
           << _sigma.at<float>(240,320);
#endif

  emit mean(_newRunningMean);
  emit variance(_variance);
  emit sd(_sigma);
}
