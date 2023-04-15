#ifndef DOUBLESPINBOXCONTROL_H
#define DOUBLESPINBOXCONTROL_H

#include <QDoubleSpinBox>

class DoubleSpinBoxControl : public QDoubleSpinBox
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Double Spin Box")
  Q_CLASSINFO("directory","Qt/Controls")
  Q_CLASSINFO("slots","in(double)")
  Q_CLASSINFO("signals","out(double)")

  Q_PROPERTY(QString caption READ caption WRITE setCaption)

public:

  Q_INVOKABLE explicit DoubleSpinBoxControl(QWidget *parent = nullptr);

  virtual ~DoubleSpinBoxControl() {}

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

  void onValueChanged(double value);

};

#endif // DOUBLESPINBOXCONTROL_H
