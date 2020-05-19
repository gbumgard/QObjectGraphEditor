#include "ObjectGraphNode.h"

#include <QApplication>
#include <QGraphicsProxyWidget>

#include <QMetaObject>
#include <QMetaClassInfo>
#include <QMetaProperty>

#include <QWidget>
#include <QGraphicsSceneMouseEvent>
#include <QStyleOptionGraphicsItem>
#include <QPainter>
#include <QWidget>
#include <QList>
#include <QMarginsF>
#include <QRgba64>
#include <QDrag>

#include <QDebug>

#include "GraphicsItemMimeData.h"

#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QGraphicsDropShadowEffect>

#include "ObjectGraph.h"
#include "ObjectGraphEdge.h"

#include "SlotConnectionPoint.h"
#include "SignalConnectionPoint.h"

#include "SizeGripper.h"

ObjectGraphNode::ObjectGraphNode(QObject* object, QGraphicsItem* parent)
  : QGraphicsWidget(parent)
  , _object(object)
  , _proxy(nullptr)
{
  setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable);
  setAttribute(Qt::WA_DeleteOnClose);

  connect(_object,&QObject::objectNameChanged,this,&ObjectGraphNode::onObjectNameChanged);

  _pen = QPen(Qt::white,2);
  _brush = QBrush(Qt::gray);
  setPen(_pen);
  setBrush(_brush);

  setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
  buildNode();
  updateNodeLayout();
}

ObjectGraphNode::~ObjectGraphNode() {
  if (scene()) scene()->removeItem(this);
}

bool ObjectGraphNode::read(QDataStream &in) {

  QMap<int,QVariant> nodeProperties;
  in >> nodeProperties;

  const QMetaObject* metaObj = metaObject();
  for (auto index : nodeProperties.keys()) {
    metaObj->property(index).write(this,nodeProperties[index]);
  }
  return true;
}

bool ObjectGraphNode::write(QDataStream &out) const {
  QMap<int,QVariant> nodeProperties;
  const QMetaObject* metaObj = metaObject();
  for (int i = 0; i < metaObj->propertyCount(); i++) {
    QMetaProperty property = metaObj->property(i);
    if (property.userType() > QMetaType::UnknownType &&
        property.userType() < QMetaType::User &&
        property.isValid() &&
        property.isReadable() &&
        property.isWritable() &&
        property.isStored()) {
      nodeProperties.insert(i,property.read(this));
    }
  }
  out << nodeProperties;
  return true;
}


ObjectGraph* ObjectGraphNode::graph() const {
  return dynamic_cast<ObjectGraph*>(scene());
}

ObjectModel* ObjectGraphNode::model() const {
  return graph()->model();
}

int ObjectGraphNode::objectId() const {
  return model()->getObjectId(object());
}

void ObjectGraphNode::setPen(const QPen& pen) {
  if (_pen != pen) {
    _pen = pen;
    updateNodeLayout();
  }
}

void ObjectGraphNode::setBrush(const QBrush& brush) {
  if (_brush != brush) {
    _brush = brush;
    updateNodeLayout();
  }
}

void ObjectGraphNode::setCaption(const QString& title) {
  if (_caption->text() != title) {
    _caption->setText(title);
    updateNodeLayout();
  }
}

void ObjectGraphNode::setStatus(const QString& status) {
  if (_status->toHtml() != status) {
    _status->setHtml(status);
    updateNodeLayout();
  }
}

void ObjectGraphNode::onObjectNameChanged(const QString& name) {
  setCaption(name);
}

void ObjectGraphNode::bindToSignalConnectionPoint(int signalIndex, ObjectGraphEdge* edge) {
  if (_signalConnectionPoints.contains(signalIndex)) {
    SignalConnectionPoint* cp = _signalConnectionPoints[signalIndex];
    edge->setSignalPosition(cp->scenePos());
    connect(cp,&ConnectionPoint::scenePositionChanged,edge,&ObjectGraphEdge::setSignalPosition);
  }
}

void ObjectGraphNode::bindToSlotConnectionPoint(int slotIndex, ObjectGraphEdge* edge) {
  if (_slotConnectionPoints.contains(slotIndex)) {
    SlotConnectionPoint* cp = _slotConnectionPoints[slotIndex];
    edge->setSlotPosition(cp->scenePos());
    connect(cp,&ConnectionPoint::scenePositionChanged,edge,&ObjectGraphEdge::setSlotPosition);
  }
}

bool ObjectGraphNode::resizable() const {
  if (_proxy && _proxy->widget()) {
    QWidget* w = _proxy->widget();
    QSizePolicy policy = w->sizePolicy();
    return policy.horizontalPolicy() != QSizePolicy::Fixed
           || policy.verticalPolicy() != QSizePolicy::Fixed;
  }
  return false;
}

void ObjectGraphNode::updateNodeLayout() {

  /*
   *     ++--------------------------------------++
   *     ||+------------------------------------+||
   *     |||            Node Caption            |||
   *     ||+------------------------------------+||
   *     ||              +---------+             ||
   *     |0 <slot name>  |         | <sign name> 0|
   *     ||              |         |             ||
   *     |0 <slot name>  |         | <sign name> 0|
   *     ||              |         |             ||
   *     |0 <slot name>  |         | <sign name> 0|
   *     ||              +---------+             ||
   *     ||+--------------------------------*   +||
   *     ||| Status Message...              |  /|||
   *     ||+--------------------------------+ +-+||
   *     ++--------------------------------------++
   *
   */

  int maxMethodCount = qMax(_slotTextItems.count(),_signalTextItems.count());
  QSizeF methodCaptionSize = maxMethodCaptionSize();

  //qreal proxyWidth = 0, proxyHeight = 0;
  qreal minProxyWidth = 0, minProxyHeight = 0;
  //qreal preferredProxyWidth = 0, preferredProxyHeight = 0;
  if (_proxy && _proxy->widget()) {
    //_proxy->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Preferred);
    minProxyWidth = _proxy->minimumWidth();
    minProxyHeight = _proxy->minimumHeight();
    //preferredProxyWidth = _proxy->preferredWidth();
    //preferredProxyHeight = _proxy->preferredHeight();
    //proxyWidth = _proxy->size().width();
    //proxyHeight = _proxy->size().height();
    qDebug() << Q_FUNC_INFO
             << "proxy-size" << _proxy->size()
             << "proxy-preferred" << _proxy->preferredSize()\
             << "proxy-min:" << _proxy->minimumSize()
             << "widget-size:" << _proxy->widget()->size();
  }

  qreal penSpacing = pen().width()/2.0;
  qreal padRadius = ConnectionPoint::RADIUS;
  qreal textWidth = methodCaptionSize.width();
  qreal textHeight = methodCaptionSize.height();
  qreal textSpacingX = padRadius;
  qreal textSpacingY = 4;
  qreal proxySpacing = padRadius;
  qreal lineHeight = textHeight + textSpacingY;

  QFont captionFont = font();
  captionFont.setBold(true);
  captionFont.setPointSize(font().pointSize()+2);
  _caption->setFont(captionFont);

  QFont statusFont = font();
  statusFont.setBold(true);
  statusFont.setPointSize(font().pointSize()+2);
  _status->setFont(captionFont);

  qreal headerWidth = penSpacing + 2* textSpacingY + _caption->boundingRect().width();
  qreal footerWidth = penSpacing + 2* textSpacingY + _status->boundingRect().width();

  qreal boxLeft = penSpacing/2.0 + padRadius;
  qreal minBoxWidth = qMax(padRadius
                           + penSpacing / 2.0
                           + (_slotConnectionPoints.count() > 0 ? (textSpacingX + textWidth) : 0)
                           + (_proxy ? minProxyWidth + 2*proxySpacing : 0)
                           + (_signalConnectionPoints.count() > 0 ? (textWidth + textSpacingX) : 0)
                           + penSpacing / 2.0
                           + padRadius,
                           qMax(headerWidth,footerWidth));

  qreal fixedWidth = size().width()-penSpacing - 2*padRadius;
  qreal boxWidth = qMax(fixedWidth,minBoxWidth);

  qreal boxRight = boxLeft + boxWidth;
  qreal boxHCenter = boxLeft + boxWidth / 2;

  qreal boxTop = penSpacing;
  qreal minBoxHeight = penSpacing / 2.0
                       + textSpacingY
                       + lineHeight
                       + qMax(lineHeight * (maxMethodCount), (_proxy ? minProxyHeight + 2* proxySpacing : 0))
                       + textSpacingY
                       + lineHeight
                       + penSpacing / 2.0;

  qreal fixedHeight = size().height()-2*penSpacing;
  qreal boxHeight = qMax(minBoxHeight,fixedHeight);

  qDebug() << "min-width:"
           << minBoxWidth
           << "fixed-width:"
           << fixedWidth
           << "box-width:"
           << boxWidth
           << "min-height:"
           << minBoxHeight
           << "fixed-height:"
           << fixedHeight
           << "box-height:"
           << boxHeight;

  qreal boxBottom = boxTop + boxHeight;

  if (_proxy && _proxy->widget()) {
    qreal availProxyHeight = boxHeight - penSpacing - 2 * textSpacingY - 2 * lineHeight - 2 * proxySpacing;
    qreal availProxyWidth = boxWidth
                            - padRadius
                            - penSpacing / 2.0
                            - (_slotConnectionPoints.count() > 0 ? (textSpacingX + textWidth) : 0)
                            - (_proxy ? 2*proxySpacing : 0)
                            - (_signalConnectionPoints.count() > 0 ? (textWidth + textSpacingX) : 0)
                            - penSpacing / 2.0
                            - padRadius;
    //if (availProxyWidth > preferredProxyWidth || availProxyHeight > preferredProxyHeight) {
      qDebug() << "avail-proxy-width:" << availProxyWidth << "height:" << availProxyHeight;
      _proxy->resize(availProxyWidth,availProxyHeight);
    //}
    _proxy->setPos(QPointF(boxLeft
                           + padRadius
                           + penSpacing/2.0
                           + (_slotConnectionPoints.count() > 0 ? (textSpacingX + textWidth) : 0)
                           + proxySpacing,
                           lineHeight + penSpacing / 2 + textSpacingY + proxySpacing));
  }

  QRectF rect(QPointF(boxLeft,boxTop),QPointF(boxRight,boxBottom));

  QPainterPath path;
  path.addRoundedRect(rect,5.0,5.0);

  _rect->setPath(path);

  _caption->setPos(boxHCenter - _caption->boundingRect().width()/2,
                 penSpacing + textSpacingY);

  _status->setPos(boxHCenter - _status->boundingRect().width()/2,
                  boxBottom - penSpacing - textSpacingY - lineHeight);

  if (resizable()) {
    if (!_sizeGripper)
      _sizeGripper = new SizeGripper(QSizeF(20.0,20.0),this);
    _sizeGripper->setPen(QPen(Qt::white,2));
    _sizeGripper->setBrush(QBrush(Qt::darkGray));
    _sizeGripper->setPos(boxRight-25,boxBottom-25);
  }
  else {
    if (_sizeGripper) {
      _sizeGripper->hide();
    }
  }

  qreal methodTextOffsetY = penSpacing + 2 * textSpacingY + lineHeight;

  if (_slotTextItems.count()) {
    QPointF position(boxLeft + padRadius + penSpacing + textSpacingX, methodTextOffsetY);
    for (auto textItem : _slotTextItems) {
      textItem->setBrush(Qt::white);
      textItem->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }

  if (_signalTextItems.count()){
    QPointF position(boxRight - padRadius - penSpacing - textSpacingX, methodTextOffsetY);
    for (auto textItem : _signalTextItems) {
      qreal width = textItem->boundingRect().size().width();
      textItem->setBrush(Qt::white);
      textItem->setPos(QPointF(position.x()-width,position.y()));
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }

  qreal methodPadOffsetY = penSpacing + 2 * textSpacingY + lineHeight + (lineHeight-padRadius)/2;
  if (_slotConnectionPoints.count()) {
    QPointF position(boxLeft,methodPadOffsetY);
    for (auto connectionPoint : _slotConnectionPoints) {
      connectionPoint->setPen(pen());
      connectionPoint->setBrush(brush());
      connectionPoint->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }

  if (_signalConnectionPoints.count()){
    QPointF position(boxRight,methodPadOffsetY);
    for (auto connectionPoint : _signalConnectionPoints) {
      connectionPoint->setPen(pen());
      connectionPoint->setBrush(brush());
      connectionPoint->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }
  prepareGeometryChange();
  updateBoundingRect();
  updateShape();
  updateGeometry();
}

QSizeF ObjectGraphNode::maxMethodCaptionSize() const {
  QRectF bounds;
  for (auto textItem : _slotTextItems) {
    bounds = bounds.united(textItem->boundingRect());
  }
  for (auto textItem : _signalTextItems) {
    bounds = bounds.united(textItem->boundingRect());
  }
  return bounds.size();
}

void ObjectGraphNode::buildNode() {

  const QMetaObject* metaObject = _object->metaObject();

  // Build temporary map of class info entries
  QMap<QString,QString> classInfo;
  for (int i = 0; i < metaObject->classInfoCount(); i++) {
    const QMetaClassInfo metaClassInfo = metaObject->classInfo(i);
    classInfo.insert(metaClassInfo.name(),metaClassInfo.value());
  }

  _rect = new QGraphicsPathItem(this);
  _rect->setPen(QPen(Qt::white,2));
  _rect->setBrush(QBrush(QRgba64::fromRgba(0x80,0x80,0x80,0xCC)));

  _caption = new QGraphicsSimpleTextItem(classInfo.value("class-alias",_object->metaObject()->className()),this);
  _caption->setPen(QPen(Qt::NoPen));
  _caption->setBrush(Qt::white);

  _status = new QGraphicsTextItem(this);
  //_status->setHtml("<span style='color:lightgreen;'>OK</span>");

  if (resizable())
    _sizeGripper = new SizeGripper(QSizeF(20.0,20.0),this);
  else
    _sizeGripper = nullptr;

  QWidget* widget = dynamic_cast<QWidget*>(_object);
  if (widget) {
    _proxy = new QGraphicsProxyWidget(this);
    _proxy->setWidget(widget);
    _proxy->setZValue(_rect->zValue()+1);
    //_proxy->widget()->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    connect(_proxy,&QGraphicsProxyWidget::widthChanged,this,&ObjectGraphNode::onProxySizeChanged);
  }

  // Use QMetaObject.methodOffset for derived class unless a dynamic property specifies otherwise
  int methodOffset = metaObject->methodOffset();
  //QVariant v = _object->property("methodOffset");
  //if (v.isValid()) methodOffset = v.toInt();

  for (int i = methodOffset; i < metaObject->methodCount(); i++) {

    QMetaMethod metaMethod = metaObject->method(i);

    if (metaMethod.access() == QMetaMethod::Public) {
      if (metaMethod.methodType() == QMetaMethod::Slot) {

        QString name = classInfo.value(metaMethod.methodSignature(),metaMethod.name());

        SlotConnectionPoint* connectionPoint = new SlotConnectionPoint(metaMethod,this);
        connectionPoint->setData(0,metaMethod.methodSignature());
        _slotConnectionPoints.insert(i,connectionPoint);

        QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(name,this);
        textItem->setPen(QPen(Qt::NoPen));
        textItem->setBrush(Qt::white);
        _slotTextItems.insert(i,textItem);
      }
      else if (metaMethod.methodType() == QMetaMethod::Signal) {

        QString name = classInfo.value(metaMethod.methodSignature(),metaMethod.name());

        SignalConnectionPoint* connectionPoint = new SignalConnectionPoint(metaMethod,this);
        connectionPoint->setData(0,metaMethod.methodSignature());
        _signalConnectionPoints.insert(i,connectionPoint);

        QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(name,this);
        textItem->setPen(QPen(Qt::NoPen));
        textItem->setBrush(Qt::white);
        _signalTextItems.insert(i,textItem);
      }
    }
  }
}

void ObjectGraphNode::updateBoundingRect() {
  QRectF rect;
  rect = rect.united(QRectF(_rect->boundingRect().topLeft()+_rect->pos(),_rect->boundingRect().size()));
  for (auto connectionPoint : _slotConnectionPoints) {
    rect = rect.united(QRectF(connectionPoint->boundingRect().topLeft() + connectionPoint->pos(),
                              connectionPoint->boundingRect().size()));
  }
  for (auto connectionPoint : _signalConnectionPoints) {
    rect = rect.united(QRectF(connectionPoint->boundingRect().topLeft() + connectionPoint->pos(),
                              connectionPoint->boundingRect().size()));
  }
  _boundingRect = rect;
}

void ObjectGraphNode::updateShape() {
  QPainterPath path = _rect->shape();
  path.translate(_rect->pos());
  _shape = path;
  for (auto connectionPoint : _slotConnectionPoints) {
    QPainterPath path = connectionPoint->shape();
    path.translate(_rect->pos());
    _shape.addPath(path);
  }
  for (auto connectionPoint : _signalConnectionPoints) {
    QPainterPath path = connectionPoint->shape();
    path.translate(_rect->pos());
    _shape.addPath(path);
  }
  _shape = path;
}

void ObjectGraphNode::onProxySizeChanged() {
  qDebug() << Q_FUNC_INFO;
  updateNodeLayout();
}

QPainterPath ObjectGraphNode::shape() const {
  return _shape;
}

QRectF ObjectGraphNode::boundingRect() const {
  return _boundingRect;
}

QSizeF ObjectGraphNode::sizeHint(Qt::SizeHint which, const QSizeF &constraint) const {
  QSizeF hint;
  switch (which) {
    case Qt::MinimumSize:
      hint = QSizeF(0,0);
      break;
    case Qt::PreferredSize:
    {
      hint = _boundingRect.size();
      break;
    }
    case Qt::MaximumSize:
    default:
      hint = QSizeF(16777215,16777215);
      break;
  }
  if (constraint.width() != -1) hint.setWidth(constraint.width());
  if (constraint.height() != -1) hint.setHeight(constraint.height());
  return hint;
}

void ObjectGraphNode::closeEvent(QCloseEvent *event) {
  (void)event;
  for (auto pad : _slotConnectionPoints) {
    pad->disconnect();
  }
  for (auto pad : _signalConnectionPoints) {
    pad->disconnect();
  }
}

QVariant ObjectGraphNode::itemChange(GraphicsItemChange change, const QVariant &value) {
  qDebug() << Q_FUNC_INFO << change << value;
  if (change == QGraphicsItem::ItemSelectedChange) {
    qDebug() << (value.toBool() ? "selected" : "not selected");
    QPen pen(QColor(value.toBool() ? "orange" : "white"), 2);

    setPen(pen);
    _rect->setPen(pen);

    for (auto connectionPoint : _slotConnectionPoints) {
      connectionPoint->setPen(pen);
    }
    for (auto connectionPoint : _signalConnectionPoints) {
      connectionPoint->setPen(pen);
    }

    update();
  }

  return QGraphicsObject::itemChange(change,value);
}

void ObjectGraphNode::resizeEvent(QGraphicsSceneResizeEvent *event) {
  updateNodeLayout();
  event->accept();
}

void ObjectGraphNode::mousePressEvent(QGraphicsSceneMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    setZValue(ObjectGraph::topZValue());
  }
  QGraphicsObject::mousePressEvent(event);
}

#if 0
void ObjectGraphNode::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  QGraphicsObject::mouseMoveEvent(event);
}

void ObjectGraphNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {
  event->ignore();
  QGraphicsObject::mouseReleaseEvent(event);
}

void ObjectGraphNode::hoverEnterEvent(QGraphicsSceneHoverEvent *event) {
  event->ignore();
  QGraphicsObject::hoverEnterEvent(event);
}

void ObjectGraphNode::hoverLeaveEvent(QGraphicsSceneHoverEvent *event) {
  event->ignore();
  QGraphicsObject::hoverLeaveEvent(event);
}

void ObjectGraphNode::dragEnterEvent(QGraphicsSceneDragDropEvent *event) {
  event->ignore();
  QGraphicsObject::dragEnterEvent(event);
}

void ObjectGraphNode::dragLeaveEvent(QGraphicsSceneDragDropEvent *event) {
  event->ignore();
  QGraphicsObject::dragLeaveEvent(event);
}

void ObjectGraphNode::dragMoveEvent(QGraphicsSceneDragDropEvent *event) {
  event->ignore();
  QGraphicsObject::dragMoveEvent(event);
}

void ObjectGraphNode::dropEvent(QGraphicsSceneDragDropEvent *event) {
  event->ignore();
  QGraphicsObject::dropEvent(event);
}

#endif
