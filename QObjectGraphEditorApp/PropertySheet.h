#ifndef PROPERTYSHEETWIDGET_H
#define PROPERTYSHEETWIDGET_H

#include <QWidget>

class PropertySheetPrivate;
class QtProperty;

class PropertySheet : public QWidget
{
  Q_OBJECT

public:

  PropertySheet(QWidget *parent = 0);
  ~PropertySheet();

  void setObject(QObject *object);
  QObject *object() const;

signals:

  void slotValueChanged(QtProperty*, const QVariant& oldValue, const QVariant& newValue);

private:

  PropertySheetPrivate *d_ptr;
  Q_DECLARE_PRIVATE(PropertySheet)
  Q_DISABLE_COPY(PropertySheet)
  Q_PRIVATE_SLOT(d_func(), void slotValueChanged(QtProperty *, const QVariant &))
};

#endif // PROPERTYSHEETWIDGET_H
