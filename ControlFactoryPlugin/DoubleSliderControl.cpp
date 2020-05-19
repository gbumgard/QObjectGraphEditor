#include "DoubleSliderControl.h"
#include "ControlFactoryPlugin.h"

REGISTER_CLASS(DoubleSliderControl)

DoubleSliderControl::DoubleSliderControl(QWidget *parent)
  : QSlider(parent)
{
  setProperty("methodOffset",QAbstractSlider::staticMetaObject.methodOffset());
  _minimumValue = QSlider::minimum();
  _maximumValue = QSlider::maximum();
  connect(this,&QSlider::valueChanged,this,&DoubleSliderControl::onValueChanged);
}

double DoubleSliderControl::value() const {
  double ratio = double(QSlider::value() - QSlider::minimum()) / double(QSlider::maximum() - QSlider::minimum());
  return ratio * (_maximumValue - _minimumValue) + _minimumValue;
}

void DoubleSliderControl::setMinimum(double dvalue) {
  if (dvalue != _minimumValue) {
    _minimumValue = dvalue;
    updateDoubleValue();
  }
}

void DoubleSliderControl::setMaximum(double dvalue) {
  if (dvalue != _maximumValue) {
    _maximumValue = dvalue;
    updateDoubleValue();
  }
}

void DoubleSliderControl::in(double dvalue) {
  if (dvalue >= _minimumValue && dvalue <= _maximumValue) {
    double ratio = (dvalue - _minimumValue) / (_maximumValue - _minimumValue);
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
  emit out(value());
}
