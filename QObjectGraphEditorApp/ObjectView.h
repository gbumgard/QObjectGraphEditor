#ifndef OBJECTGRAPHVIEW_H
#define OBJECTGRAPHVIEW_H

#include <QWidget>
#include <QGraphicsView>
#include <QWheelEvent>
#include <QToolButton>

class SimpleObject;
class ObjectGraphNode;

class ObjectView
    : public QGraphicsView
{

  Q_OBJECT

public:

  ObjectView(QWidget* parent = nullptr);
  ObjectView(QGraphicsScene* scene, QWidget* parent = nullptr);

  virtual ~ObjectView();

  bool load(const QString& fileName);
  bool save(const QString& fileName) const;


public slots:

  void onRecenterPressed();

private slots:

  void onRubberBandChanged(QRect rubberBandRect, QPointF fromScenePoint, QPointF toScenePoint);

protected:

  bool read(QDataStream& in);

  bool write(QDataStream& out) const;

  void wheelEvent (QWheelEvent* event) override;

  void resizeEvent(QResizeEvent *event) override;

  virtual void contextMenuEvent(QContextMenuEvent *event) override;

  void mousePressEvent(QMouseEvent *event) override;

  void mouseReleaseEvent(QMouseEvent *event) override;

private:

  QToolButton* _recenterButton;

  QRect _selectionArea;

};

#endif // OBJECTGRAPHVIEW_H
