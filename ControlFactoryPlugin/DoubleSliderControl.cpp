#include "DoubleSliderControl.h"
#include "ControlFactoryPlugin.h"

REGISTER_CLASS(DoubleSliderControl)

DoubleSliderControl::DoubleSliderControl(QWidget *parent)
  : QSlider(parent)
{
  setProperty("methodOffset",QAbstractSlider::staticMetaObject.methodOffset());
  _scaleMin = QSlider::minimum();
  _scaleMax = QSlider::maximum();
  connect(this,&QSlider::valueChanged,this,&DoubleSliderControl::onValueChanged);
}

double DoubleSliderControl::scaleValue() const {
  double ratio = double(QSlider::value() - QSlider::minimum()) / double(QSlider::maximum() - QSlider::minimum());
  return ratio * (_scaleMax - _scaleMin) + _scaleMin;
}

void DoubleSliderControl::setScaleMin(double dvalue) {
  if (dvalue != _scaleMin && dvalue != _scaleMax) {
      _scaleMin = dvalue;
    //updateDoubleValue();
    emit scaleMinChanged(dvalue);
  }
}

void DoubleSliderControl::setScaleMax(double dvalue) {
  if (dvalue != _scaleMax && dvalue != _scaleMin) {
    _scaleMax = dvalue;
    //updateDoubleValue();
    emit scaleMaxChanged(dvalue);
  }
}

void DoubleSliderControl::in(double dvalue) {
  if (dvalue >= _scaleMin && dvalue <= _scaleMax && _scaleMin != _scaleMax) {
    double ratio = (dvalue - _scaleMin) / (_scaleMax - _scaleMin);
    int sliderValue = ratio * (QSlider::maximum() - QSlider::minimum()) + QSlider::minimum();
    if (sliderValue != QSlider::value()) {
      QSlider::setValue(sliderValue);
      emit out(dvalue);
    }
  }
}

void DoubleSliderControl::onValueChanged(int) {
  updateDoubleValue();
}

void DoubleSliderControl::updateDoubleValue() {
  emit out(scaleValue());
}
