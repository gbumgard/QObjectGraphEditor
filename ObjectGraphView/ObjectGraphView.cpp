#include "ObjectGraphView.h"
#include <QStyle>
#include <QIcon>
#include <QMimeData>

#include <QDebug>

ObjectGraphView::ObjectGraphView(QWidget* parent)
  : QGraphicsView(parent)
  , _numScheduledScalings(0)
  , _scaleSteps(0.0)
  , _recenterButton(new QToolButton(this))
{
  connect(_recenterButton,&QToolButton::pressed,this,&ObjectGraphView::onRecenterPressed);
  _recenterButton->setIcon(_recenterButton->style()->standardIcon(QStyle::SP_TitleBarNormalButton));
  setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
}

ObjectGraphView::ObjectGraphView(QGraphicsScene* scene, QWidget* parent)
  : QGraphicsView(scene,parent)
{

}

void ObjectGraphView::onRecenterPressed() {
  fitInView(scene()->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void ObjectGraphView::resizeEvent(QResizeEvent *event) {

  _recenterButton->move(10,10);
  QGraphicsView::resizeEvent(event);
}

void ObjectGraphView::wheelEvent(QWheelEvent* event)
{
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  QPoint numDegrees = event->angleDelta();
  static const double stepScale = 1.15;

  if (!numDegrees.isNull()) {
    int numSteps = numDegrees.ry() / 120;
    double scaleFactor = 0;
    if (numSteps < 0) {
      scaleFactor = pow(1.0/stepScale,-numSteps);
    }
    else {
      scaleFactor = pow(stepScale,numSteps);
    }
    scale(scaleFactor,scaleFactor);
  }
}
