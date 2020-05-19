#ifndef SLIDERCONTROL_H
#define SLIDERCONTROL_H

#include <QSlider>

class SliderControl : public QSlider
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Slider")
  Q_CLASSINFO("directory","Qt/Controls")

public:

  Q_INVOKABLE explicit SliderControl(QWidget* parent = nullptr);

  virtual ~SliderControl() {}

signals:

  void out(int value);

public slots:

  void in(int value);

private slots:

  void onValueChanged(int value);

private:

};

#endif // SLIDERCONTROL_H
