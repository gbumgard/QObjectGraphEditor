#ifndef OBJECTGRAPHVIEW_H
#define OBJECTGRAPHVIEW_H

#include <QWidget>
#include <QGraphicsView>
#include <QWheelEvent>
#include <QToolButton>
#include "objectgraphview_global.h"

class Object;

class OBJECTGRAPHVIEWSHARED_EXPORT ObjectGraphView
    : public QGraphicsView
{

  Q_OBJECT

public:

  ObjectGraphView(QWidget* parent = nullptr);
  ObjectGraphView(QGraphicsScene* scene, QWidget* parent = nullptr);

private slots:

  void onRecenterPressed();

protected:

  void wheelEvent (QWheelEvent* event);

  void resizeEvent(QResizeEvent *event) override;

  int _numScheduledScalings;
  int _scaleSteps;
  QToolButton* _recenterButton;

};

#endif // OBJECTGRAPHVIEW_H
