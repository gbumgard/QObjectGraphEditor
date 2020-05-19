#include "KinectV1DepthFilter.h"

#include "OpenCvFactoryPlugin.h"

#include <QDateTime>
#include <math.h>
#include <QMetaObject>
#include <QMetaMethod>
#include <iostream>
#include <iomanip>

REGISTER_CLASS(KinectV1DepthFilter)

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


KinectV1DepthFilter::KinectV1DepthFilter(QObject *parent)
  : QObject(parent)
  , _timeIntervalMs(1000)
  , _minimumGain(.0001)
  , _minimumVelocity(0.0)
  , _maximumVelocity(1.0)
{
}

void KinectV1DepthFilter::setTimeInterval(double timeIntervalMs) {
  if (timeIntervalMs > 0.0 && timeIntervalMs <= 2000.0) {
    _timeIntervalMs = (qint64)timeIntervalMs;
    qint64 currentEpochTimeMs = QDateTime::currentMSecsSinceEpoch();
    while(!_rawSampleQueue.empty() && (currentEpochTimeMs - _rawSampleQueue.front().epochTimestampMs) > _timeIntervalMs) {
      _rawSampleQueue.pop_front();
    }
  }
}

void KinectV1DepthFilter::gain(double gain) {
  if (gain <= 1.0 && gain >= 0.0)
    _minimumGain = gain;
}

void KinectV1DepthFilter::setMinimumVelocity(double velocity) {
  if (velocity >= 0.0) {
    if (velocity > _maximumVelocity)
      _maximumVelocity = velocity;
    _minimumVelocity = velocity;
  }
}

void KinectV1DepthFilter::setMaximumVelocity(double velocity) {
  if (velocity >= 0.0) {
    if (velocity < _minimumVelocity)
      _minimumVelocity = velocity;
    _maximumVelocity = velocity;
  }
}

void KinectV1DepthFilter::in(const cv::Mat &mat) {

  qint64 currentEpochTimeMs = QDateTime::currentMSecsSinceEpoch();

  cv::Mat newMeasurement = mat.clone();

  Sample newSample = { currentEpochTimeMs, newMeasurement };

  // First sample is simply queued and emitted.
  if (_rawSampleQueue.empty()
      || _filteredSample.depthFrame.empty()
      || _filteredSample.depthFrame.size() != mat.size()) {

    _rawSampleQueue.clear();

    _rawSampleQueue.push_back(newSample);

    _filteredSample = newSample;

    emit out(newMeasurement);

    return;
  }

  int newMeasurementVal = 0;
  int lastMeasurementVal = 0;
  int lastFilteredMeasurementVal = 0;
  int firstMeasurementVal = 0;
  int shortDeltaVal = 0;
  int longDeltaVal = 0;
  int filteredDeltaVal = 0;
  double normalizedFilteredDeltaVal = 0;
  double averageVelocityVal = 0;
  double normalizedAverageVelocityVal = 0;
  double maxVal = 0;
  int filteredMeasurementVal = 0;
  double gainVal = 0;
  double radiansVal = 0;


  // Get delta from most recent unfiltered sample.
  cv::Mat shortDelta(newMeasurement.size(),CV_16S);
  Sample lastSample = _rawSampleQueue.back();
  cv::Mat lastMeasurement = lastSample.depthFrame;
  //qint64 shortDeltaInterval = currentEpochTimeMs - lastSample.epochTimestampMs;

  for(int row = 0; row < newMeasurement.rows; row++)
  {
    const uint16_t* newMeasurementRow = newMeasurement.ptr<uint16_t>(row);
    const uint16_t* lastMeasurementRow = lastMeasurement.ptr<uint16_t>(row);
    for(int col = 0; col < newMeasurement.cols; col++) {
      shortDelta.at<int16_t>(row,col) = (int)newMeasurementRow[col] - (int)lastMeasurementRow[col];
    }
  }

  // Queue the new sample for future filter passes.
  _rawSampleQueue.push_back(newSample);

  // Get delta from oldest unfiltered sample
  cv::Mat longDelta(newMeasurement.size(),CV_16S);
  Sample firstSample = _rawSampleQueue.front();
  cv::Mat firstMeasurement = firstSample.depthFrame;
  qint64 longDeltaInterval = currentEpochTimeMs - firstSample.epochTimestampMs;

  for(int row = 0; row < longDelta.rows; row++)
  {
    const uint16_t* newMeasurementRow = newMeasurement.ptr<uint16_t>(row);
    const uint16_t* firstMeasurementRow = firstMeasurement.ptr<uint16_t>(row);
    for(int col = 0; col < longDelta.cols; col++) {
      longDelta.at<int16_t>(row,col) = (int)newMeasurementRow[col] - (int)firstMeasurementRow[col];
    }
  }

  // Remove the oldest sample if its age is greater than the current filter time interval.
  if (longDeltaInterval >= _timeIntervalMs) {
    _rawSampleQueue.pop_front();
  }

  // Get delta from most recent unfiltered sample
  cv::Mat filteredDelta(newMeasurement.size(),CV_16S);
  cv::Mat normalizedFilteredDelta(filteredDelta.size(),CV_32F);
  cv::Mat lastFilteredMeasurement = _filteredSample.depthFrame;
  //qint64  filterDeltaInterval = currentEpochTimeMs - _filteredSample.epochTimestampMs;

  for(int row = 0; row < filteredDelta.rows; row++)
  {
    const uint16_t* newMeasurementRow = newMeasurement.ptr<uint16_t>(row);
    const uint16_t* lastFilteredMeasurementRow = lastFilteredMeasurement.ptr<uint16_t>(row);
    for(int col = 0; col < filteredDelta.cols; col++) {
      uint16_t depth = newMeasurementRow[col];
      int16_t delta = depth - lastFilteredMeasurementRow[col];
      filteredDelta.at<int16_t>(row,col) = delta;
      normalizedFilteredDelta.at<float>(row,col) = fabs((float)delta) / getKinectV1SDTable()[depth & 0x1FFF];
    }
  }

  // Compute average velocity
  cv::Mat averageVelocity(longDelta.size(),CV_32F);

  double minVelocity = 10000.0, maxVelocity = 0.0;

  for(int row = 0; row < averageVelocity.rows; row++)
  {
    const int16_t* longDeltaRow = longDelta.ptr<int16_t>(row);
    for(int col = 0; col < averageVelocity.cols; col++) {
      double velocity = fabs((float)longDeltaRow[col]) / longDeltaInterval;
      averageVelocity.at<float>(row,col) = velocity;
      if (velocity > maxVelocity) maxVelocity = velocity;
      else if (velocity < minVelocity) minVelocity = velocity;
    }
  }


  //double velocityRange = _maximumVelocity - _minimumVelocity;
  double velocityRange = maxVelocity - minVelocity;

  // Compute normalized average velocity
  cv::Mat normalizedAverageVelocity(averageVelocity.size(),CV_32F);

  for(int row = 0; row < normalizedAverageVelocity.rows; row++)
  {
    const float* averageVelocityRow = averageVelocity.ptr<float>(row);
    for(int col = 0; col < normalizedAverageVelocity.cols; col++) {
      normalizedAverageVelocity.at<float>(row,col) = velocityRange != 0 ? (averageVelocityRow[col] - minVelocity) / velocityRange : 1.0;
    }
  }

  // Take maximum of normalized velocity or filter delta
  cv::Mat max(normalizedAverageVelocity.size(),CV_32F);

  for(int row = 0; row < normalizedAverageVelocity.rows; row++)
  {
    const float* normalizedFilteredDeltaRow = normalizedFilteredDelta.ptr<float>(row);
    const float* normalizedAverageVelocityRow = normalizedAverageVelocity.ptr<float>(row);
    for(int col = 0; col < normalizedAverageVelocity.cols; col++) {
      max.at<float>(row,col) = fmax(normalizedFilteredDeltaRow[col],normalizedAverageVelocityRow[col]);
    }
  }

  // Compute the filtered output
  cv::Mat filteredMeasurement(newMeasurement.size(),CV_16U);

  int sampleRow = 240, sampleCol = 320;

  for(int row = 0; row < filteredMeasurement.rows; row++)
  {
    const int16_t* shortDeltaRow = shortDelta.ptr<int16_t>(row);
    const int16_t* filteredDeltaRow = filteredDelta.ptr<int16_t>(row);
    const uint16_t* newMeasurementRow = newMeasurement.ptr<uint16_t>(row);
    const uint16_t* lastFilteredMeasurementRow = lastFilteredMeasurement.ptr<uint16_t>(row);
    const float* maxRow = max.ptr<float>(row);


    for(int col = 0; col < filteredMeasurement.cols; col++) {

      double gain = 0;
      double radians = 0;

      if (fabs(shortDeltaRow[col]) > getKinectV1SDTable()[newMeasurementRow[col] & 0x1FFF])  {

        filteredMeasurement.at<uint16_t>(row,col) = newMeasurementRow[col];

      }
      else {

        if (maxRow[col] <= 1.0) {
          radians = maxRow[col] * M_PI - M_PI/2.0;
          //gain = _minimumGain + .5 * (1.0 - _minimumGain) * (sin(radians)); // + 1.0);
          //gain = _minimumGain;
          gain = .005;
          filteredMeasurement.at<uint16_t>(row,col) = (1.0 - gain) * lastFilteredMeasurementRow[col] + gain * newMeasurementRow[col];
        }
        else {
          if (shortDeltaRow[col] != 0) {
            gain = filteredDeltaRow[col] / shortDeltaRow[col];
          }
          else {
            gain = 0;
          }
          filteredMeasurement.at<uint16_t>(row,col) = (uint16_t)(lastFilteredMeasurementRow[col] + gain * shortDeltaRow[col]);
        }

      }

      if (row == sampleRow && col == sampleCol) {
        gainVal = gain;
        radiansVal = radians;
      }

    }
  }


  newMeasurementVal = newMeasurement.at<uint16_t>(sampleRow,sampleCol);
  lastMeasurementVal = lastMeasurement.at<uint16_t>(sampleRow,sampleCol);
  shortDeltaVal = shortDelta.at<int16_t>(sampleRow,sampleCol);
  longDeltaVal = longDelta.at<int16_t>(sampleRow,sampleCol);
  firstMeasurementVal = firstMeasurement.at<uint16_t>(sampleRow,sampleCol);
  filteredDeltaVal = filteredDelta.at<int16_t>(sampleRow,sampleCol);
  lastFilteredMeasurementVal = lastFilteredMeasurement.at<uint16_t>(sampleRow,sampleCol);
  normalizedFilteredDeltaVal = normalizedFilteredDelta.at<float>(sampleRow,sampleCol);
  averageVelocityVal = averageVelocity.at<float>(sampleRow,sampleCol);
  normalizedAverageVelocityVal = normalizedAverageVelocity.at<float>(sampleRow,sampleCol);
  maxVal = max.at<float>(sampleRow,sampleCol);
  filteredMeasurementVal = filteredMeasurement.at<uint16_t>(sampleRow,sampleCol);

  // Save result for next round
  _filteredSample.epochTimestampMs = currentEpochTimeMs;
  _filteredSample.depthFrame = filteredMeasurement;

  std::cout << std::showpoint << std::setprecision(4)
            << "Zm=" << std::setw(5) << newMeasurementVal
            << " Zf=" << std::setw(5) << filteredMeasurementVal
            << " Zm(t-1)=" << std::setw(6) << lastMeasurementVal
            << " Zf(t-1)=" << std::setw(6) << lastFilteredMeasurementVal
            << " Zm(T)=" << std::setw(6) << firstMeasurementVal
            << " σ(Z)=" << std::setw(6) << getKinectV1SDTable()[newMeasurementVal]
            << " ΔZ(t-1)=" << std::setw(4) << shortDeltaVal
            << " ΔZ(T)=" << std::setw(4) << longDeltaVal
            << " ΔZf(t-1)=" << std::setw(4) << filteredDeltaVal
            << " ^ΔZf(t-1)=" << std::setprecision(4) << std::setw(8) << normalizedFilteredDeltaVal
            //<< " fmin-v=" << std::setw(6) << minVelocity
            << " fmax-v=" << std::setprecision(4) << std::setw(6) << maxVelocity
            << " aver-v=" << std::setw(8) << averageVelocityVal
            << " norm-v=" << std::setw(8) << normalizedAverageVelocityVal
            << " m=" << std::setw(6) << maxVal
            << " g=" << std::setw(6) << gainVal
            << " rad=" << std::setw(6) << radiansVal
            << std::endl << std::flush;

  emit out(filteredMeasurement);

}
