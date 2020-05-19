#ifndef OBJECTGRAPHVIEW_H
#define OBJECTGRAPHVIEW_H

#include <QWidget>
#include <QGraphicsView>
#include <QWheelEvent>
#include <QToolButton>

class Object;

class ObjectView
    : public QGraphicsView
{

  Q_OBJECT

public:

  ObjectView(QWidget* parent = nullptr);
  ObjectView(QGraphicsScene* scene, QWidget* parent = nullptr);

  bool load(const QString& fileName);
  bool save(const QString& fileName) const;

public slots:

  void onRecenterPressed();

protected:

  bool read(QDataStream& in);
  bool write(QDataStream& out) const;

  void wheelEvent (QWheelEvent* event);

  void resizeEvent(QResizeEvent *event) override;

  void mouseMoveEvent(QMouseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void dragEnterEvent(QDragEnterEvent *event) override;
  void dragLeaveEvent(QDragLeaveEvent *event) override;
  void dropEvent(QDropEvent *event) override;
  void dragMoveEvent(QDragMoveEvent *event) override;

private:

  int _numScheduledScalings;
  int _scaleSteps;
  QToolButton* _recenterButton;

};

#endif // OBJECTGRAPHVIEW_H
