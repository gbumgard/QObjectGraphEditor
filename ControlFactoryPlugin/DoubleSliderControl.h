#ifndef DOUBLESLIDERCONTROL_H
#define DOUBLESLIDERCONTROL_H

#include <QSlider>
#include <QApplication>
#include <QThread>
#include <QDebug>

class DoubleSliderControl : public QSlider
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Double Slider")
  Q_CLASSINFO("directory","Qt/Controls")

  Q_PROPERTY(double value READ value WRITE setValue NOTIFY out)
  Q_PROPERTY(double minimum READ minimum WRITE setMinimum)
  Q_PROPERTY(double maximum READ maximum WRITE setMaximum)

  Q_PROPERTY(QString caption READ caption WRITE setCaption)

public:

  Q_INVOKABLE explicit DoubleSliderControl(QWidget* parent = nullptr);

  virtual ~DoubleSliderControl() {}

  void setValue(double value) { in(value); }

  double value() const;

  void setMinimum(double dvalue);

  double minimum() const {
    return _minimumValue;
  }

  void setMaximum(double dvalue);

  double maximum() const {
    return _maximumValue;
  }

  QString caption() const {
    return QObject::objectName();
  }

  void setCaption(const QString& caption) {
    QObject::setObjectName(caption);
  }

signals:

  void out(double value);

public slots:

  void in(double value);

private slots:

  void onValueChanged(int);

private:

  void updateDoubleValue();

  double _minimumValue;
  double _maximumValue;

};

#endif // DOUBLESLIDERCONTROL_H
