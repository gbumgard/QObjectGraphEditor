#ifndef CHECKBOXCONTROL_H
#define CHECKBOXCONTROL_H

#include <QObject>
#include <QCheckBox>

class CheckBoxControl : public QCheckBox
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Check Box")
  Q_CLASSINFO("directory","Qt/Controls")
  Q_CLASSINFO("slots","in(bool)")
  Q_CLASSINFO("signals","out(bool)")

public:

  Q_INVOKABLE explicit CheckBoxControl(QWidget* parent = nullptr);

  virtual ~CheckBoxControl() {}

signals:

  void out(bool value);

public slots:

  void in(bool value);

private slots:

  void onStateChanged(int value);


};
#endif // CHECKBOXCONTROL_H
