#include "RunningStatistics.h"

#include "OpenCvFactoryPlugin.h"
#include <iostream>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <QDebug>

REGISTER_CLASS(RunningStatistics)

#if 0
static double depthAdjustedSd(double x) {
  return 1.3973214E-6 * double(x*x) - 0.001299821 * (double)x + 0.6893571;
}
#endif

RunningStatistics::RunningStatistics(QObject *parent)
  : QObject(parent)
  , _min(std::numeric_limits<double>::lowest())
  , _max(std::numeric_limits<double>::max())
{
}

RunningStatistics::~RunningStatistics() {
}

void RunningStatistics::clear() {
  _count = 0;
  _oldMean = 0;
  _newMean = 0;
  _oldVarianceSquared = 0;
  _newVarianceSquared = 0;
}

void RunningStatistics::in(const MatEvent &input) {

  if (input.mat().size() != _count.size()) {
    _count = cv::Mat(input.mat().size(),CV_32F,cv::Scalar(0));
    _oldMean = cv::Mat(input.mat().size(),CV_64F,cv::Scalar(0.0));
    _newMean = cv::Mat(input.mat().size(),CV_64F,cv::Scalar(0.0));
    _oldVarianceSquared = cv::Mat(input.mat().size(),CV_64F,cv::Scalar(0.0));
    _newVarianceSquared = cv::Mat(input.mat().size(),CV_64F,cv::Scalar(0.0));
  }

  cv::Mat sample;
  input.mat().convertTo(sample,CV_64F);

  cv::Mat inRangeMask;
  cv::Mat aboveMinMask = input.mat() >= _min;
  cv::Mat belowMaxMask = input.mat() <= _max;
  inRangeMask = aboveMinMask & belowMaxMask;

  // Mask for pixels that have not been initialized (count == 0)
  cv::Mat notInitializedMask = _count == 0;

  // Increment the counters for all pixels corresponding to sample pixels that are in-range.
  cv::add(_count,cv::Scalar(1),_count,inRangeMask);

  // Mask to select pixels that have not been initialized that are also in-range.
  cv::Mat initializationMask = notInitializedMask & inRangeMask;

  // Initialize pixels with count == 1 and pixel value in range
  sample.copyTo(_oldMean,initializationMask);
  sample.copyTo(_newMean,initializationMask);
  _oldVarianceSquared.setTo(cv::Scalar(0),initializationMask);

  // Mask for pixels with count > 1 (have been initialized)
  cv::Mat initializedMask = ~notInitializedMask;

  // Mask to select pixels with counts > 1 that are also in-range.
  cv::Mat updateMask = initializedMask & inRangeMask;

  cv::Mat oldDelta;
  cv::subtract(sample, _oldMean, oldDelta, updateMask, CV_64F);

  cv::Mat meanStep;
  cv::divide(oldDelta, _count, meanStep, 1.0, CV_64F);

  cv::add(_oldMean, meanStep, _newMean, updateMask, CV_64F);

  cv::Mat newDelta;
  cv::subtract(sample, _newMean, newDelta, updateMask, CV_64F);

  cv::Mat varianceSquaredStep;
  cv::multiply(oldDelta, newDelta, varianceSquaredStep, 1.0, CV_64F);

  cv::add(_oldVarianceSquared, varianceSquaredStep, _newVarianceSquared, updateMask, CV_64F);

  _oldMean = _newMean.clone();
  _oldVarianceSquared = _newVarianceSquared.clone();

  cv::Mat finalMean;
  finalMean.setTo(_newMean,initializedMask);

  cv::Mat previousCount = _count - 1;

  cv::Mat variance;
  cv::divide(_newVarianceSquared, previousCount, variance, 1.0, CV_64F);

  // Fix division by zero for pixels that have no history
  variance.setTo(cv::Scalar(0),notInitializedMask);

  cv::Mat standardDeviation;
  cv::sqrt(variance,standardDeviation);

  cv::Mat meanOut;
  _newMean.convertTo(meanOut, input.mat().depth());

  emit mean(MatEvent(meanOut,input.timestamp()));
  emit var(MatEvent(variance,input.timestamp()));
  emit std(MatEvent(standardDeviation,input.timestamp()));
  emit update(MatEvent(updateMask,input.timestamp()));

}
