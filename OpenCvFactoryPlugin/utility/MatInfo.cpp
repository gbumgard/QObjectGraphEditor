#include "MatInfo.h"

#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(MatInfo)

MatInfo::MatInfo(QObject* parent)
  : QObject(parent)
{
}

void MatInfo::in(const cv::Mat &mat) {
  emit cols(mat.cols);
  emit rows(mat.rows);
  emit type(mat.type());
  emit channels(mat.channels());
  emit depth(mat.depth());

  double minVal, maxVal;
  cv::minMaxLoc(mat,&minVal,&maxVal);
  emit min(minVal);
  emit max(maxVal);
}
