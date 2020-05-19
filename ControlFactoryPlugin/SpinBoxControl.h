#ifndef SPINBOXCONTROL_H
#define SPINBOXCONTROL_H

#include <QSpinBox>

class SpinBoxControl : public QSpinBox
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Spin Box")
  Q_CLASSINFO("directory","Qt/Controls")

public:

  Q_INVOKABLE explicit SpinBoxControl(QWidget *parent = nullptr);

  virtual ~SpinBoxControl() {}

signals:

  void out(int value);

public slots:

  void in(int value);

private slots:

  void onValueChanged(int value);

};

#endif // SPINBOXCONTROL_H
