#include "ChangeCommitFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <math.h>
#include <iostream>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

REGISTER_CLASS(ChangeCommitFilter)

ChangeCommitFilter::ChangeCommitFilter(QObject *parent)
  : QObject(parent)
  , _size(0,0)
  , _startCount(1)
  , _startThreshold(1.0)
{
}

void ChangeCommitFilter::startCount(int startCount) {
  if (startCount >= 0) {
    _startCount = startCount;
  }
}

void ChangeCommitFilter::startThreshold(qreal startThreshold) {
  if (startThreshold >= 0.0) {
    _startThreshold = startThreshold;
  }
}

void ChangeCommitFilter::in(const cv::Mat &mat) {

  //if (mat.empty() || mat.depth() != CV_8U || mat.depth() != CV_32F) return;

  // First sample is simply queued and emitted.
  if (mat.size() != _size) { // || _filteredInput.empty() || mat.depth() != _filteredInput.depth()) {

    _size = mat.size();

    _currentCount = cv::Mat(_size,CV_16U,cv::Scalar(0));
    _filteredInput = cv::Mat(_size,mat.depth());
    mat.copyTo(_filteredInput);

    //emit out(_filteredInput);

    return;
  }

  // Create mask that identifies cells with counts > 0
  cv::Mat currentCountdownMask;
  cv::threshold(_currentCount,currentCountdownMask,0,255,cv::THRESH_BINARY);
  if (currentCountdownMask.depth() != CV_8U) {
    currentCountdownMask.convertTo(currentCountdownMask,CV_8U);
  }

  emit countingMask(currentCountdownMask);

  //cv::imshow("Counting Mask",countingMask);

  // Measure abs difference between new input and current filtered output
  cv::Mat currentDifference;
  cv::absdiff(mat,_filteredInput,currentDifference);

  //cv::imshow("abs diff",difference);
  emit absDifference(currentDifference);

  // Create mask for cells whose absolute difference is > _startThreshold
  cv::Mat thresholdMask;
  cv::threshold(currentDifference,thresholdMask,_startThreshold,255,cv::THRESH_BINARY);
  if (thresholdMask.depth() != CV_8U) {
    thresholdMask.convertTo(thresholdMask,CV_8U);
  }

  //cv::imshow("threshold mask",thresholdMask);
  emit startThresholdMask(thresholdMask);

  // Create mask for cells that need to start count down - cell needing countdown minus cells already in countdown
  cv::Mat newCountdownMask;
  cv::subtract(thresholdMask,currentCountdownMask,newCountdownMask);

  emit startCountdownMask(newCountdownMask);

  _currentCount.setTo(cv::Scalar(_startCount),newCountdownMask);

  //cv::imshow("countdown mask", countdownMask);
  //cv::imshow("current count",_currentCount);

  // Create filter mask by finding all cells not in or needing countdown
  /*
  cv::Mat filterUpdateMask;
  cv::threshold(currentDifference,filterUpdateMask,_startThreshold,255,CV_THRESH_BINARY_INV);
  if (filterUpdateMask.depth() != CV_8U) {
    filterUpdateMask.convertTo(filterUpdateMask,CV_8U);
  }
  cv::Mat filterUpdateMask;
  cv::bitwise_and(currentCountdownMask,newCountdownMask,filterUpdateMask);
  filterUpdateMask = 255 - filterUpdateMask;
  */
  cv::Mat filterUpdateMask;
  cv::threshold(_currentCount,filterUpdateMask,1,255,cv::THRESH_BINARY_INV);
  if (filterUpdateMask.depth() != CV_8U) {
    filterUpdateMask.convertTo(filterUpdateMask,CV_8U);
  }

  emit filterMask(filterUpdateMask);

  //std::cout << "update filtered output" << std::endl;
  mat.copyTo(_filteredInput,filterUpdateMask);

  //cv::imshow("filter mask", filterMask);

  // Decrement all non-zero counts
  //std::cout << "Decrement all non-zero counts"
  //          << _currentCount.size() << " depth=" << _currentCount.depth() << " chan=" << _currentCount.channels()
  //          << countingMask.size() << " depth=" << countingMask.depth() << " chan=" << countingMask.channels() << std::endl;
  cv::subtract(_currentCount,cv::Scalar(1),_currentCount,currentCountdownMask);
  //std::cout << "emit filtered output" << std::endl;

  emit currentCount(_currentCount);

  emit out(_filteredInput);
}
