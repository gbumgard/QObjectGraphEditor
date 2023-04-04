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

  Q_PROPERTY(double minimum READ minimum WRITE setMinimum NOTIFY minimumChanged)
  Q_PROPERTY(double maximum READ maximum WRITE setMaximum NOTIFY maximumChanged)
  Q_PROPERTY(double scaleMin READ scaleMin WRITE setScaleMin NOTIFY scaleMinChanged)
  Q_PROPERTY(double scaleMax READ scaleMax WRITE setScaleMax NOTIFY scaleMaxChanged)
  Q_PROPERTY(double scaleValue READ scaleValue WRITE setScaleValue NOTIFY out)
  Q_PROPERTY(QString caption READ caption WRITE setCaption NOTIFY captionChanged)

public:

  Q_INVOKABLE explicit DoubleSliderControl(QWidget* parent = nullptr);

  virtual ~DoubleSliderControl() {}

  int minimum() const {
      return QSlider::minimum();
  }

  int maximum() const {
      return QSlider::minimum();
  }

  double scaleValue() const;

  double scaleMin() const {
      return _scaleMin;
  }

  double scaleMax() const {
      return _scaleMax;
  }


  QString caption() const {
    return QObject::objectName();
  }

  void setCaption(const QString& caption) {
    QObject::setObjectName(caption);
  }

signals:

  void out(double value);
  void minimumChanged(int value);
  void maximumChanged(int value);
  void scaleMinChanged(double);
  void scaleMaxChanged(double);
  void scaleValueChanged(double);
  void captionChanged(QString&);

public slots:

  void setMinimum(int value) { QSlider::setMinimum(value); emit minimumChanged(value);}
  void setMaximum(int value) { QSlider::setMaximum(value); emit maximumChanged(value);}
  void setScaleValue(double value) { in(value); }
  void setScaleMin(double dvalue);
  void setScaleMax(double dvalue);

  void in(double value);

private slots:

  void onValueChanged(int);

private:

  void updateDoubleValue();

  double _scaleMin;
  double _scaleMax;

};

#endif // DOUBLESLIDERCONTROL_H
