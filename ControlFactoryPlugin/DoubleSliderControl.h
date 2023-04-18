#ifndef DOUBLESLIDERCONTROL_H
#define DOUBLESLIDERCONTROL_H

#include <QSlider>
#include <QApplication>
#include <QThread>
#include <QDebug>

#ifndef Q_MOC_RUN
#  define PRIVATE_SIGNAL_TAG
#endif


class DoubleSliderControl : public QSlider
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Double Slider")
  Q_CLASSINFO("directory","Qt/Controls")
  Q_CLASSINFO("slots","in(double),max(double),min(double)")
  Q_CLASSINFO("signals","out(double)")

  //Q_PROPERTY(double minimum READ minimum WRITE setMinimum NOTIFY minimumChanged)
  //Q_PROPERTY(double maximum READ maximum WRITE setMaximum NOTIFY maximumChanged)
  Q_PROPERTY(double min READ min WRITE min NOTIFY minChanged)
  Q_PROPERTY(double max READ max WRITE max NOTIFY maxChanged)
  Q_PROPERTY(double scaleValue READ scaleValue WRITE setScaleValue NOTIFY out)
  Q_PROPERTY(QString caption READ caption WRITE caption NOTIFY captionChanged)

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

  double min() const {
      return _scaleMin;
  }

  double max() const {
      return _scaleMax;
  }


  QString caption() const {
    return QObject::objectName();
  }

  void caption(const QString& caption) {
    QObject::setObjectName(caption);
  }

signals:

  void out(double value);

  void minimumChanged(int value);
  void maximumChanged(int value);
  void captionChanged(QString&);
  void minChanged(double);
  void maxChanged(double);
  void scaleValueChanged(double);


public slots:

  void in(double value);

  void min(double dvalue);
  void max(double dvalue);

  void setScaleValue(double value) { in(value); }

  private slots:

  void setMinimum(int value) { QSlider::setMinimum(value); emit minimumChanged(value);}
  void setMaximum(int value) { QSlider::setMaximum(value); emit maximumChanged(value);}

  void onValueChanged(int);

private:

  void updateDoubleValue();

  double _scaleMin;
  double _scaleMax;

};

#endif // DOUBLESLIDERCONTROL_H
