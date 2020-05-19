#include "MaxFilter.h"
#include "OpenCvFactoryPlugin.h"

#include <math.h>
#include <iostream>
#include <iomanip>

REGISTER_CLASS(MaxFilter)

MaxFilter::MaxFilter(QObject *parent)
  : QObject(parent)
  , _maxValue(0.0)
{
}

void MaxFilter::maxValue(qreal maxValue) {
  _maxValue = maxValue;
}

void MaxFilter::in(const cv::Mat &mat) {

  if (mat.empty()) return;

  emit out(cv::max(mat,_maxValue));
}
