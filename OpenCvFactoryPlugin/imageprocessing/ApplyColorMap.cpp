#include "ApplyColorMap.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(ApplyColorMap)

ApplyColorMap::ApplyColorMap(QObject* parent)
  : QObject(parent)
  , _colorMap(COLORMAP_RAINBOW)
{
}

void ApplyColorMap::colorMap(ColorMap colorMap) {
  if (colorMap != _colorMap) {
    _colorMap = colorMap;
    update();
  }
}

void ApplyColorMap::in(const cv::Mat &mat) {
  _input = mat;
  update();
}

void ApplyColorMap::update() {
  cv::Mat dst;
  cv::applyColorMap(_input,dst,_colorMap);
  emit out(dst);
}
