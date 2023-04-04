#ifndef NODE_H
#define NODE_H

#include <QGraphicsWidget>
#include <QPainterPath>

#include <QPen>
#include <QBrush>

#include <QMetaMethod>

#include <QMap>
#include <QMenu>
#include <QGraphicsSceneContextMenuEvent>
#include <QUuid>

#include <QDebug>

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

  Q_PROPERTY(QString caption READ caption WRITE setCaption NOTIFY captionChanged)
  Q_PROPERTY(int statusCode READ statusCode CONSTANT)
  Q_PROPERTY(QString statusMessage READ statusMessage CONSTANT)
  Q_PROPERTY(QPen pen READ pen WRITE setPen NOTIFY penChanged)
  Q_PROPERTY(QBrush brush READ brush WRITE setBrush NOTIFY brushChanged)

public:

  ObjectGraphNode(QObject* object, QGraphicsItem* parent = nullptr);

  virtual ~ObjectGraphNode();

  ObjectGraph* graph() const;

  ObjectModel* model() const;

  QObject* object() const { return _object; }

  QUuid uuid() const { return objectUuid(); }

  QUuid objectUuid() const;

  QPen pen() const { return _pen; }
  void setPen(const QPen& pen);

  QBrush brush() const { return _brush; }
  void setBrush(const QBrush& brush);

  QString caption() const { return _caption->text(); }
  void setCaption(const QString& caption);

  int statusCode() const { return _statusCode; }
  QString statusMessage() const { return _status->toPlainText(); }

  void setStatus(int statusCode, const QString& statusMessage);

  QPainterPath shape() const override;
  QRectF boundingRect() const override;

  void bindToSignalConnectionPoint(const QString &signalSignature, ObjectGraphEdge* edge);
  void bindToSlotConnectionPoint(const QString& slotIndex, ObjectGraphEdge* edge);

  void  contextMenuEvent(QGraphicsSceneContextMenuEvent* e) override;

signals:

  void onMoved(const QPointF offset);
  void captionChanged(const QString& caption);
  void penChanged(const QPen& pen);
  void brushChanged(const QBrush& brush);

public slots:

private slots:

  void onObjectDestroyed();

  void onObjectNameChanged(const QString& name);

  void updateNodeLayout();

  void onProxySizeChanged();

  void onContextHelp() const;

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
  void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;
  void mouseMoveEvent(QGraphicsSceneMouseEvent *event) override;
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

  ObjectGraph* _graph;

  int _statusCode;

  QGraphicsPathItem* _rect;
  QGraphicsSimpleTextItem* _caption;
  QGraphicsTextItem* _status;
  SizeGripper* _sizeGripper;

  QVector<QMetaMethod> _slotMethodMap;
  QVector<QMetaMethod> _signalMethodMap;

  QMap<QString,SlotConnectionPoint*> _slotConnectionPoints;
  QMap<QString,SignalConnectionPoint*> _signalConnectionPoints;

  QMap<QString,QGraphicsSimpleTextItem*> _slotTextItems;
  QMap<QString,QGraphicsSimpleTextItem*> _signalTextItems;

  QRectF _geometry;
  QRectF _boundingRect;
  QPainterPath _shape;

  QPen _pen;
  QBrush _brush;

};

#endif // NODE_H
