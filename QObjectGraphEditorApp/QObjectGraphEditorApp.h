#ifndef QOBJECTGRAPHEDITORAPP_H
#define QOBJECTGRAPHEDITORAPP_H

#include <QMainWindow>
#include <QStandardItemModel>
#include <QUndoStack>

#include "ObjectModel.h"

class QtProperty;
class ObjectGraph;

namespace Ui {
  class QObjectGraphEditorApp;
}

class QObjectGraphEditorApp : public QMainWindow
{
  Q_OBJECT

public:

  explicit QObjectGraphEditorApp(QWidget *parent = 0);
  ~QObjectGraphEditorApp();

protected:

  static constexpr const char* GRAPH_MIME_TYPE = "application/x-graphite-grf";

  void loadObjectFactoryPlugins();

  void setStyles();

  void populateObjectTreeView();

  bool open(const QString& fileName);
  bool save(const QString& fileName);

  virtual void closeEvent(QCloseEvent* event);

private slots:

  void newModel();
  void openModel();
  void openRecentModel(bool);
  void saveModel();
  void saveModelAs();
  void recentModels();
  void closeModel();
  void exitApp();

  void about();
  void aboutPlugins();

  void toggleFullScreen();

  void onCutAction();
  void onCopyAction();
  void onPasteAction();
  void onDeleteAction();
  void onSelectAllAction();

  void onSceneSelectionChanged();

  void slotValueChanged(QtProperty*, const QVariant&, const QVariant&);

private:

  Ui::QObjectGraphEditorApp *ui;

  QStandardItemModel* _factoryClassTree;

  QUndoStack _commandStack;

  ObjectGraph* _graph;

};

#endif // QOBJECTGRAPHEDITORAPP_H
