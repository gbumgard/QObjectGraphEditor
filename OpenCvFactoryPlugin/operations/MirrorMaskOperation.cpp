#include "MirrorMaskOperation.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(MirrorMaskOperation)

MirrorMaskOperation::MirrorMaskOperation(QObject* parent)
  : QObject(parent)
{
}

void MirrorMaskOperation::mask(const cv::Mat &mat) {
  _mask = mat;
  update();
}

void MirrorMaskOperation::update() {
  cv::Mat dst;
  cv::threshold(_mask,dst,0,255,cv::THRESH_BINARY_INV);
  emit original(_mask);
  emit inverse(dst);
}
