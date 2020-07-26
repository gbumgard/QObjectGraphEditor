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
#include <QString>
#include <QVector>
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
#include <sstream>
#include <vector>

ObjectGraphNode::ObjectGraphNode(QObject* object, QGraphicsItem* parent)
  : QGraphicsWidget(parent)
  , _object(object)
  , _proxy(nullptr)
  , _statusCode(0)
{

  qDebug() << "\n" << Q_FUNC_INFO;

  setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable | ItemSendsScenePositionChanges);
  setAttribute(Qt::WA_DeleteOnClose);

  connect(_object,&QObject::objectNameChanged,this,&ObjectGraphNode::onObjectNameChanged);
  connect(_object,&QObject::destroyed,this,&ObjectGraphNode::onObjectDestroyed);

  _pen = QPen(Qt::white,2);
  _brush = QBrush(Qt::gray);
  setPen(_pen);
  setBrush(_brush);

  setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
  buildNode();
  setZValue(ObjectGraph::topNodeZValue());
  updateNodeLayout();
}

ObjectGraphNode::~ObjectGraphNode() {
  qDebug() << "\n" << Q_FUNC_INFO;
  if (scene()) scene()->removeItem(this);
}


ObjectGraph* ObjectGraphNode::graph() const {
  return _graph;
}

ObjectModel* ObjectGraphNode::model() const {
  return graph()->model();
}

QUuid ObjectGraphNode::objectUuid() const {
  return model()->objectUuid(_object);
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

void ObjectGraphNode::setStatus(int statusCode, const QString& statusMessage) {
  if (_statusCode != statusCode || _status->toPlainText() != statusMessage) {
    _statusCode = statusCode;

    QString message("<span style='color:%1;'>%2</span>");
    if (_statusCode == ObjectModel::STATUS_OK) {
      _status->setHtml(message.arg("lime").arg(statusMessage));
      _rect->setBrush(QBrush(QRgba64::fromRgba(0x80,0x80,0x80,0xCC)));
    }
    else {
      _status->setHtml(message.arg("yellow").arg(statusMessage));
      _rect->setBrush(QBrush(QRgba64::fromRgba(0xC0,0x80,0x80,0xCC)));
    }


    updateNodeLayout();
  }
}

void  ObjectGraphNode::contextMenuEvent(QGraphicsSceneContextMenuEvent* e) {
  qDebug() << Q_FUNC_INFO << e;
  //QMenu* menu = new QMenu;
  //QAction* helpAction = menu->addAction("help");
  QMenu menu;
  QAction* helpAction = menu.addAction("Context Help",this,SLOT(onContextHelp()));
  (void)helpAction;
  menu.exec(e->screenPos());
}

void ObjectGraphNode::onContextHelp() const {
  qDebug() << Q_FUNC_INFO;
}

void ObjectGraphNode::onObjectDestroyed() {
  qDebug() << Q_FUNC_INFO;
  _object = nullptr;
}

void ObjectGraphNode::onObjectNameChanged(const QString& name) {
  setCaption(name);
}

void ObjectGraphNode::bindToSignalConnectionPoint(const QString& signalSignature, ObjectGraphEdge* edge) {

  qDebug() << Q_FUNC_INFO << signalSignature;

  if (_signalConnectionPoints.contains(signalSignature)) {

    SignalConnectionPoint* cp = _signalConnectionPoints[signalSignature];
    edge->setSignalPosition(cp->scenePos());
    connect(cp,&ConnectionPoint::scenePositionChanged,edge,&ObjectGraphEdge::setSignalPosition);

  }
}

void ObjectGraphNode::bindToSlotConnectionPoint(const QString &slotSignature, ObjectGraphEdge* edge) {

  qDebug() << Q_FUNC_INFO << slotSignature;

  if (_slotConnectionPoints.contains(slotSignature)) {

    SlotConnectionPoint* cp = _slotConnectionPoints[slotSignature];
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

  //qDebug() << Q_FUNC_INFO;

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

  if (_slotMethodMap.count()) {
    QPointF position(boxLeft + padRadius + penSpacing + textSpacingX, methodTextOffsetY);
    for (auto metaMethod : _slotMethodMap) {
      auto textItem =_slotTextItems[metaMethod.methodSignature()];
      textItem->setBrush(Qt::white);
      textItem->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }

#if 0
  if (_slotTextItems.count()) {
    QPointF position(boxLeft + padRadius + penSpacing + textSpacingX, methodTextOffsetY);
    for (auto textItem : _slotTextItems) {
      textItem->setBrush(Qt::white);
      textItem->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }
#endif

  if (_signalMethodMap.count()) {
    QPointF position(boxRight - padRadius - penSpacing - textSpacingX, methodTextOffsetY);
    for (auto metaMethod : _signalMethodMap) {
      auto textItem =_signalTextItems[metaMethod.methodSignature()];
      qreal width = textItem->boundingRect().size().width();
      textItem->setBrush(Qt::white);
      textItem->setPos(QPointF(position.x()-width,position.y()));
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }

#if 0
  if (_signalTextItems.count()){
    QPointF position(boxRight - padRadius - penSpacing - textSpacingX, methodTextOffsetY);
    for (auto textItem : _signalTextItems) {
      qreal width = textItem->boundingRect().size().width();
      textItem->setBrush(Qt::white);
      textItem->setPos(QPointF(position.x()-width,position.y()));
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }
#endif

  qreal methodPadOffsetY = penSpacing + 2 * textSpacingY + lineHeight + (lineHeight-padRadius)/2;
  if (_slotConnectionPoints.count()) {
    QPointF position(boxLeft,methodPadOffsetY);
    for (auto metaMethod : _slotMethodMap) {
      auto connectionPoint = _slotConnectionPoints[metaMethod.methodSignature()];
      connectionPoint->setPen(pen());
      connectionPoint->setBrush(brush());
      connectionPoint->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }
#if 0
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
#endif

  if (_signalConnectionPoints.count()){
    QPointF position(boxRight,methodPadOffsetY);
    for (auto metaMethod : _signalMethodMap) {
      auto connectionPoint = _signalConnectionPoints[metaMethod.methodSignature()];
      connectionPoint->setPen(pen());
      connectionPoint->setBrush(brush());
      connectionPoint->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }

#if 0
  if (_signalConnectionPoints.count()){
    QPointF position(boxRight,methodPadOffsetY);
    for (auto connectionPoint : _signalConnectionPoints) {
      connectionPoint->setPen(pen());
      connectionPoint->setBrush(brush());
      connectionPoint->setPos(position);
      position = QPointF(position.x(),position.y()+lineHeight);
    }
  }
#endif

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

  // Use QMetaObject.methodOffset for derived class unless a dynamic property specifies otherwise
  int methodOffset = metaObject->methodOffset();
  QVariant v = _object->property("methodOffset");
  if (v.isValid() && v.toInt() < metaObject->methodCount()) {
    methodOffset = v.toInt();
  }

  if (classInfo.keys().contains("slot-order")) {
    std::string slotList = classInfo["slot-order"].toUtf8().constData();
    if (!slotList.empty()) {
      qDebug() << "slot-order" << slotList.c_str();
      std::stringstream tokenizer(slotList);
      std::string token;
      while(std::getline(tokenizer,token,',')) {
        int index = metaObject->indexOfSlot(token.c_str());
        if (index != -1 && metaObject->method(index).access() == QMetaMethod::Public) {
          qDebug() << "adding" << index << token.c_str();
          _slotMethodMap.push_back(metaObject->method(index));
        }
        else {
          qWarning() << "CLASSINFO slot-order entry" << token.c_str() << "is undefined or inaccessible";
        }
      }
    }
  }
  else {
    for (int index = methodOffset; index < metaObject->methodCount(); index++) {
      QMetaMethod method = metaObject->method(index);
      if (method.methodType() == QMetaMethod::Slot && method.access() == QMetaMethod::Public) {
        _slotMethodMap.push_back(method);
      }
    }
  }

  if (classInfo.keys().contains("signal-order")) {
    std::string signalList = classInfo["signal-order"].toUtf8().constData();
    if (!signalList.empty()) {
      qDebug() << "signal-order" << signalList.c_str();
      std::stringstream tokenizer(signalList);
      std::string token;
      while(std::getline(tokenizer,token,',')) {
        int index = metaObject->indexOfSignal(token.c_str());
        if (index != -1 && metaObject->method(index).access() == QMetaMethod::Public) {
          qDebug() << "adding" << index << token.c_str();
          _signalMethodMap.push_back(metaObject->method(index));
        }
        else {
          qWarning() << "CLASSINFO signal-order entry" << token.c_str() << "is undefined or inaccessible";
        }
      }
    }
  }
  else {
    for (int index = methodOffset; index < metaObject->methodCount(); index++) {
      QMetaMethod method = metaObject->method(index);
      if (method.methodType() == QMetaMethod::Signal && method.access() == QMetaMethod::Public) {
        _signalMethodMap.push_back(method);
      }
    }
  }

  qDebug() << "exposing the following slots";
  for (auto metaMethod : _slotMethodMap) {
    qDebug() << metaMethod.methodSignature();
  }

  qDebug() << "exposing the following signals";
  for (auto metaMethod : _signalMethodMap) {
    qDebug() << metaMethod.methodSignature();
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

  for (auto metaMethod : _slotMethodMap) {
    QString methodSignature = metaMethod.methodSignature();

    SlotConnectionPoint* connectionPoint = new SlotConnectionPoint(metaMethod,this);
    connectionPoint->setData(0,methodSignature);
    _slotConnectionPoints[methodSignature] = connectionPoint;

    QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(metaMethod.name(),this);
    textItem->setPen(QPen(Qt::NoPen));
    textItem->setBrush(Qt::white);
    _slotTextItems[methodSignature] = textItem;
  }

  for (auto metaMethod : _signalMethodMap) {
    QString methodSignature = metaMethod.methodSignature();

    SignalConnectionPoint* connectionPoint = new SignalConnectionPoint(metaMethod,this);
    connectionPoint->setData(0,methodSignature);
    _signalConnectionPoints[methodSignature] = connectionPoint;

    QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(metaMethod.name(),this);
    textItem->setPen(QPen(Qt::NoPen));
    textItem->setBrush(Qt::white);
    _signalTextItems[methodSignature] = textItem;
  }

#if 0
  for (int i = methodOffset; i < metaObject->methodCount(); i++) {

    QMetaMethod metaMethod = metaObject->method(i);

    if (metaMethod.access() == QMetaMethod::Public) {
      if (metaMethod.methodType() == QMetaMethod::Slot) {

        QString name = classInfo.value(metaMethod.methodSignature(),metaMethod.name());

        SlotConnectionPoint* connectionPoint = new SlotConnectionPoint(metaMethod,this);
        connectionPoint->setData(0,metaMethod.methodSignature());
        _slotConnectionPoints.insert(metaMethod.methodSignature(),connectionPoint);

        QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(name,this);
        textItem->setPen(QPen(Qt::NoPen));
        textItem->setBrush(Qt::white);
        //_slotTextItems.insert(metaMethod.methodSignature(),textItem);
      }
      else if (metaMethod.methodType() == QMetaMethod::Signal) {

        QString name = classInfo.value(metaMethod.methodSignature(),metaMethod.name());

        SignalConnectionPoint* connectionPoint = new SignalConnectionPoint(metaMethod,this);
        connectionPoint->setData(0,metaMethod.methodSignature());
        _signalConnectionPoints.insert(metaMethod.methodSignature(),connectionPoint);

        QGraphicsSimpleTextItem* textItem = new QGraphicsSimpleTextItem(name,this);
        textItem->setPen(QPen(Qt::NoPen));
        textItem->setBrush(Qt::white);
        _signalTextItems.insert(metaMethod.methodSignature(),textItem);
      }
    }
  }
#endif
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

  qDebug() << "\n" << Q_FUNC_INFO << change << value;

  switch (change) {

    case QGraphicsItem::ItemSelectedChange:
      {
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
      break;

    case QGraphicsItem::ItemSceneHasChanged:
      {

        QGraphicsScene *graph = scene();

        _graph = dynamic_cast<ObjectGraph*>(graph);
      }

      break;

    default:

      break;
  }

  return QGraphicsObject::itemChange(change,value);
}

void ObjectGraphNode::resizeEvent(QGraphicsSceneResizeEvent *event) {
  updateNodeLayout();
  event->accept();
}

void ObjectGraphNode::mousePressEvent(QGraphicsSceneMouseEvent *event) {

  qDebug() << "\n" << Q_FUNC_INFO;

  if (event->button() == Qt::LeftButton) {
    setZValue(ObjectGraph::topNodeZValue());
  }

  QGraphicsObject::mousePressEvent(event);
}


#if 0
void ObjectGraphNode::mouseReleaseEvent(QGraphicsSceneMouseEvent *event) {

  qDebug() << "\n" << Q_FUNC_INFO;

  event->ignore();
  QGraphicsObject::mouseReleaseEvent(event);
}

void ObjectGraphNode::mouseMoveEvent(QGraphicsSceneMouseEvent *event) {
  QGraphicsObject::mouseMoveEvent(event);
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
