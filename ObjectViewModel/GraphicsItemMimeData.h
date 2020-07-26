#ifndef GRAPHICSWIDGETMIMEDATA_H
#define GRAPHICSWIDGETMIMEDATA_H

#include <QObject>

#include <QObject>
#include <QMimeData>
#include <QGraphicsItem>

class GraphicsItemMimeData : public QMimeData
{
  Q_OBJECT

public:

  explicit GraphicsItemMimeData(QGraphicsItem* item,
                                const QString& mimeType = QString("application/x-graphicswidgetref"));

  virtual ~GraphicsItemMimeData() {}

  QStringList formats() const override;
  bool hasFormat(const QString& mimeType) const override;

  QGraphicsItem* item() const { return _item; }

private:

  QGraphicsItem* _item;
  QString _mimeType;

};


#endif // GRAPHICSWIDGETMIMEDATA_H
