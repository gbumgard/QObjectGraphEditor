#include "CheckBoxControl.h"
#include "ControlFactoryPlugin.h"

REGISTER_CLASS(CheckBoxControl)

CheckBoxControl::CheckBoxControl(QWidget *parent)
  : QCheckBox(parent)
{
  setProperty("methodOffset",QAbstractButton::staticMetaObject.methodOffset());
  connect(this,&QCheckBox::stateChanged,this,&CheckBoxControl::onStateChanged);
  setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
}

void CheckBoxControl::onStateChanged(int value) {
  emit out(value == Qt::Checked);
}

void CheckBoxControl::in(bool value) {
  CheckBoxControl::setCheckState(value ? Qt::Checked : Qt::Unchecked);
}
