#include "ConvertScaleAbs.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(ConvertScaleAbs)

ConvertScaleAbs::ConvertScaleAbs(QObject* parent)
  : QObject(parent)
  , _scale(1.0)
  , _offset(0.0)
{
}

void ConvertScaleAbs::scale(double scale) {
  _scale = scale;
}

void ConvertScaleAbs::offset(double offset) {
  _offset = offset;
}

void ConvertScaleAbs::in(const TaggedMat &taggedMat) {
  if (!taggedMat.first.empty()) {
    cv::Mat dst;
    cv::convertScaleAbs(taggedMat.first, dst, _scale, _offset);
    emit out(TaggedMat(dst,taggedMat.second));
  }
}
