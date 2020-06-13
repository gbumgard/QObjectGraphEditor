#include "MatInfo.h"

#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(MatInfo)

MatInfo::MatInfo(QObject* parent)
  : QObject(parent)
{
}

void MatInfo::in(const TaggedMat &taggedMat) {
  setSeq(taggedMat.second);
  setCols(taggedMat.first.cols);
  setRows(taggedMat.first.rows);
  setChannels(taggedMat.first.channels());
  setType((MatType)taggedMat.first.type());
  setDepth((MatDepth)taggedMat.first.depth());
  double minVal, maxVal;
  cv::minMaxLoc(taggedMat.first,&minVal,&maxVal);
  setMin(minVal);
  setMax(maxVal);
}
