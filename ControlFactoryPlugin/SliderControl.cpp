#include "SliderControl.h"
#include "ControlFactoryPlugin.h"

REGISTER_CLASS(SliderControl)

SliderControl::SliderControl(QWidget *parent)
  : QSlider(parent)
{
  setProperty("methodOffset",QAbstractSlider::staticMetaObject.methodOffset());
  connect(this,&QSlider::valueChanged,this,&SliderControl::onValueChanged);
  setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
}

void SliderControl::onValueChanged(int value) {
  emit out(value);
}

void SliderControl::in(int value) {
  QSlider::setValue(value);
}
