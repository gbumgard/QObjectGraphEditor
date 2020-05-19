#include "SpinBoxControl.h"
#include "ControlFactoryPlugin.h"

REGISTER_CLASS(SpinBoxControl)

SpinBoxControl::SpinBoxControl(QWidget *parent)
  : QSpinBox(parent)
{
  setProperty("methodOffset",QAbstractSpinBox::staticMetaObject.methodOffset());
  connect(this,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),this,&SpinBoxControl::onValueChanged);
}

void SpinBoxControl::onValueChanged(int value) {
  emit out(value);
}

void SpinBoxControl::in(int value) {
  QSpinBox::setValue(value);
}
