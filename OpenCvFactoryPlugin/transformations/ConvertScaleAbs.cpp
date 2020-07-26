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

void ConvertScaleAbs::in(const MatEvent &event) {
  cv::Mat output  ;
  cv::convertScaleAbs(event.mat(), output, _scale, _offset);
  emit out(MatEvent(output,event.timestamp()));
}
