#ifndef SPINBOXCONTROL_H
#define SPINBOXCONTROL_H

#include <QSpinBox>

class SpinBoxControl : public QSpinBox
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Spin Box")
  Q_CLASSINFO("directory","Qt/Controls")

  Q_PROPERTY(QString caption READ caption WRITE setCaption)

public:

  Q_INVOKABLE explicit SpinBoxControl(QWidget *parent = nullptr);

  virtual ~SpinBoxControl() {}

  QString caption() const {
    return QObject::objectName();
  }

  void setCaption(const QString& caption) {
    QObject::setObjectName(caption);
  }

signals:

  void out(int value);

public slots:

  void in(int value);

private slots:

  void onValueChanged(int value);

};

#endif // SPINBOXCONTROL_H
