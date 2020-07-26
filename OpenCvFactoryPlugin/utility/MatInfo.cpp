#include "MatInfo.h"

#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(MatInfo)

MatInfo::MatInfo(QObject* parent)
  : QObject(parent)
{
}

void MatInfo::in(const MatEvent &event) {
  setSeq(event.timestamp());
  setCols(event.mat().cols);
  setRows(event.mat().rows);
  setChannels(event.mat().channels());
  setType((MatType)event.mat().type());
  setDepth((MatDepth)event.mat().depth());
  double minVal, maxVal;
  cv::minMaxLoc(event.mat(),&minVal,&maxVal);
  setMin(minVal);
  setMax(maxVal);
}
