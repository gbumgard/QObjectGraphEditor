#include "DoubleSpinBoxControl.h"
#include "ControlFactoryPlugin.h"

REGISTER_CLASS(DoubleSpinBoxControl)

DoubleSpinBoxControl::DoubleSpinBoxControl(QWidget *parent)
  : QDoubleSpinBox(parent)
{
  setProperty("methodOffset",QAbstractSpinBox::staticMetaObject.methodOffset());
  connect(this,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),this,&DoubleSpinBoxControl::onValueChanged);
}

void DoubleSpinBoxControl::onValueChanged(double value) {
  emit out(value);
}

void DoubleSpinBoxControl::in(double value) {
  QDoubleSpinBox::setValue(value);
}
