#ifndef DOUBLESPINBOXCONTROL_H
#define DOUBLESPINBOXCONTROL_H

#include <QDoubleSpinBox>

class DoubleSpinBoxControl : public QDoubleSpinBox
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Double Spin Box")
  Q_CLASSINFO("directory","Qt/Controls")

public:

  Q_INVOKABLE explicit DoubleSpinBoxControl(QWidget *parent = nullptr);

  virtual ~DoubleSpinBoxControl() {}

signals:

  void out(double value);

public slots:

  void in(double value);

private slots:

  void onValueChanged(double value);

};

#endif // DOUBLESPINBOXCONTROL_H
