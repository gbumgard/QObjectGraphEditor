#ifndef NODE_H
#define NODE_H

#include <QGraphicsWidget>
#include <QPainterPath>

#include <QPen>
#include <QBrush>

#include <QMetaMethod>

#include <QMap>

class QGraphicsProxyWidget;
class QGraphicsSimpleTextItem;

class ObjectModel;
class ObjectGraph;
class ObjectGraphEdge;

class SlotConnectionPoint;
class SignalConnectionPoint;

class SizeGripper;

class ObjectGraphNode : public QGraphicsWidget
{

  Q_OBJECT

  Q_PROPERTY(QString caption READ caption WRITE setCaption)
  Q_PROPERTY(QString status READ status WRITE setStatus)
  Q_PROPERTY(QPen pen READ pen WRITE setPen)
  Q_PROPERTY(QBrush brush READ brush WRITE setBrush)

public:

  ObjectGraphNode(QObject* object, QGraphicsItem* parent = nullptr);

  virtual ~ObjectGraphNode();

  bool read(QDataStream& in);
  bool write(QDataStream& out) const;

  ObjectGraph* graph() const;

  ObjectModel* model() const;

  QObject* object() const { return _object; }

  int objectId() const;

  QPen pen() const { return _pen; }
  void setPen(const QPen& pen);

  QBrush brush() const { return _brush; }
  void setBrush(const QBrush& brush);

  QString caption() const { return _caption->text(); }
  void setCaption(const QString& caption);

  QString status() const { return _status->toPlainText(); }
  void setStatus(const QString& status);

  QPainterPath shape() const override;
  QRectF boundingRect() const override;

  void bindToSignalConnectionPoint(int signalIndex, ObjectGraphEdge* edge);
  void bindToSlotConnectionPoint(int slotIndex, ObjectGraphEdge* edge);

private slots:

  void onObjectNameChanged(const QString& name);
  void updateNodeLayout();

  void onProxySizeChanged();

protected:

  void buildMethodMaps();

  QSizeF maxMethodCaptionSize() const;

  void buildNode();

  bool resizable() const;

  void updateBoundingRect();

  void updateShape();

  QSizeF sizeHint(Qt::SizeHint which, const QSizeF &constraint) const override;

  void closeEvent(QCloseEvent *event) override;

  void resizeEvent(QGraphicsSceneResizeEvent *event) override;

  QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

  void mousePressEvent(QGraphicsSceneMouseEvent *event) override;

#if 0
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

  void hoverEnterEvent(QGraphicsSceneHoverEvent *event) override;
  void hoverLeaveEvent(QGraphicsSceneHoverEvent *event) override;

  void dragEnterEvent(QGraphicsSceneDragDropEvent *event) override;
  void dragLeaveEvent(QGraphicsSceneDragDropEvent *event) override;
  void dragMoveEvent(QGraphicsSceneDragDropEvent *event) override;
  void dropEvent(QGraphicsSceneDragDropEvent *event) override;
#endif

private:

  QObject* _object;
  QGraphicsProxyWidget* _proxy;

  QGraphicsPathItem* _rect;
  QGraphicsSimpleTextItem* _caption;
  QGraphicsTextItem* _status;
  SizeGripper* _sizeGripper;

  QMap<int,SlotConnectionPoint*> _slotConnectionPoints;
  QMap<int,SignalConnectionPoint*> _signalConnectionPoints;

  QMap<int,QGraphicsSimpleTextItem*> _slotTextItems;
  QMap<int,QGraphicsSimpleTextItem*> _signalTextItems;

  QRectF _geometry;
  QRectF _boundingRect;
  QPainterPath _shape;

  QPen _pen;
  QBrush _brush;
  //QPointF _lastEventPosition;

};

#endif // NODE_H
