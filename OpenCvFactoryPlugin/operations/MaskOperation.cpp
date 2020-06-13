#include "MaskOperation.h"
#include "OpenCvFactoryPlugin.h"

#include <opencv2/imgproc.hpp>

REGISTER_CLASS(MaskOperation)

MaskOperation::MaskOperation(QObject* parent)
  : AbstractOpenCvObject(parent)
{
}

void MaskOperation::in(const TaggedMat &taggedMat) {
  _input = taggedMat;
  update();
}

void MaskOperation::mask(const TaggedMat &taggedMat) {
  _mask = taggedMat;
  update();
}

void MaskOperation::update() {
  if (!_input.first.empty() && !_mask.first.empty() && _input.second == _mask.second && _input.first.size() == _mask.first.size()) {
    cv::Mat dst;
    _input.first.copyTo(dst,_mask.first > 0);
    emit out(TaggedMat(dst,_input.second));
  }
}
