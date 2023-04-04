#include "ApplyColorMap.h"
#include "ObjectModel.h"

REGISTER_CLASS(ApplyColorMap)

ApplyColorMap::ApplyColorMap(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _colorMap(COLORMAP_RAINBOW)
{
}

void ApplyColorMap::colorMap(ColorMap colorMap) {
  if (colorMap != _colorMap) {
    _colorMap = colorMap;
    update();
  }
}

void ApplyColorMap::src(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    _srcEvent = qvariant_cast<MatEvent>(variant);
    update();
  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}

void ApplyColorMap::update() {
  cv::Mat mat8UC1;
  _srcEvent.mat().convertTo(mat8UC1,CV_8UC1);
  cv::Mat output;
  cv::applyColorMap(mat8UC1,output,_colorMap);
  emit dst(QVariant::fromValue(MatEvent(output,_srcEvent.timestamp())));
}
