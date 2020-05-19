#include "QObjectGraphEditorApp.h"
#include "ui_QObjectGraphEditorApp.h"

#include <QGraphicsWidget>
#include <QDir>
#include <QPluginLoader>
#include <QGenericPlugin>
#include <QFileDialog>
#include <QMessageBox>
#include <QAction>

#include <QJsonObject>

#include <QMetaObject>
#include <QMetaMethod>
#include <QBrush>
#include <QStringLiteral>
#include <QGraphicsItem>

#include <memory>

#include "ObjectFactory.h"
#include "ObjectGraph.h"
#include "ObjectGraphNode.h"
#include "ObjectGraph.h"
#include "ObjectModel.h"

#include <QUuid>
#include <QDebug>
#include <QMimeData>

#include <QtProperty>

QObjectGraphEditorApp::QObjectGraphEditorApp(QWidget *parent)
  : QMainWindow(parent)
  , ui(new Ui::QObjectGraphEditorApp)
  , _factoryClassTree(new QStandardItemModel(this))
{
  ui->setupUi(this);

  //ui->horizontalSplitter->setStretchFactor(0,1);
  ui->horizontalSplitter->setStretchFactor(1,100);
  //ui->horizontalSplitter->setStretchFactor(2,1);

  ui->objectClasses->setModel(_factoryClassTree);

  _graph = new ObjectGraph(this);
  ObjectModel* model = new ObjectModel(this);
  _graph->setModel(model);
  _graph->setCommandStack(&_commandStack);
  ui->objectView->setScene(_graph);

  connect(_graph,&QGraphicsScene::selectionChanged,this, &QObjectGraphEditorApp::onSceneSelectionChanged);
  setStyles();

  QAction* undoAction = _commandStack.createUndoAction(this, tr("&Undo"));
  undoAction->setShortcuts(QKeySequence::Undo);

  QAction* redoAction = _commandStack.createRedoAction(this, tr("&Redo"));
  redoAction->setShortcuts(QKeySequence::Redo);

  ui->editMenu->insertAction(ui->editMenu->actions().first(),redoAction);
  ui->editMenu->insertAction(redoAction,undoAction);

  /*
  scene->open(qApp->applicationDirPath()+QDir::separator()+QString("scene.db"));

  paths << qApp->applicationDirPath()+QDir::separator()+QString("../plugins");
  scene->loadPluginObjects(paths);
 */

  QStringList paths;
  paths << qApp->applicationDirPath()+QDir::separator()+QString("../plugins");
  //paths << qApp->applicationDirPath()+QDir::separator()+QString("../../../../plugins");
  ObjectFactory::loadObjectFactoryPlugins(paths);
  qDebug() << ObjectFactory::names();
  populateObjectTreeView();

  QModelIndexList indexes = _factoryClassTree->match(_factoryClassTree->index(0,0), Qt::DisplayRole, "*", -1, Qt::MatchWildcard|Qt::MatchRecursive);
  foreach (QModelIndex index, indexes)
    ui->objectClasses->expand(index);

  connect(ui->actionNewModel,&QAction::triggered,this,&QObjectGraphEditorApp::newModel);
  connect(ui->actionOpenModel,&QAction::triggered,this,&QObjectGraphEditorApp::openModel);
  connect(ui->actionSaveModel,&QAction::triggered,this,&QObjectGraphEditorApp::saveModel);
  connect(ui->actionSaveModelAs,&QAction::triggered,this,&QObjectGraphEditorApp::saveModelAs);
  connect(ui->actionCloseModel,&QAction::triggered,this,&QObjectGraphEditorApp::closeModel);
  connect(ui->actionExit,&QAction::triggered,this,&QObjectGraphEditorApp::exitApp);
  connect(ui->actionCut,&QAction::triggered,this,&QObjectGraphEditorApp::cut);
  connect(ui->actionCopy,&QAction::triggered,this,&QObjectGraphEditorApp::copy);
  connect(ui->actionPaste,&QAction::triggered,this,&QObjectGraphEditorApp::paste);
  connect(ui->actionSelectAll,&QAction::triggered,this,&QObjectGraphEditorApp::selectAll);
  connect(ui->actionFullScreen,&QAction::triggered,this,&QObjectGraphEditorApp::toggleFullScreen);
  connect(ui->actionAbout,&QAction::triggered,this,&QObjectGraphEditorApp::about);
  connect(ui->actionAboutPlugins,&QAction::triggered,this,&QObjectGraphEditorApp::aboutPlugins);

}

QObjectGraphEditorApp::~QObjectGraphEditorApp()
{
  disconnect(qobject_cast<ObjectGraph*>(ui->objectView->scene()),&QGraphicsScene::selectionChanged,this, &QObjectGraphEditorApp::onSceneSelectionChanged);
  disconnect(ui->propertySheet,&PropertySheet::slotValueChanged,this,&QObjectGraphEditorApp::slotValueChanged);
  delete ui;
}

void QObjectGraphEditorApp::slotValueChanged(QtProperty*, const QVariant&, const QVariant&) {
  /*
  qDebug() << Q_FUNC_INFO
           << ui->propertySheet->object()->objectName()
           << property->propertyName()
           << value;
           */
}

void QObjectGraphEditorApp::loadObjectFactoryPlugins() {

  QStringList paths;
  paths << qApp->applicationDirPath()+QDir::separator()+QString("../plugins");

  ObjectFactory::loadObjectFactoryPlugins(paths);

  //qDebug() << ObjectFactory::names();
}

void QObjectGraphEditorApp::setStyles() {
}

void QObjectGraphEditorApp::populateObjectTreeView() {
  QList<QString> classNames = ObjectFactory::names();
  for (auto className : classNames) {
    QString label = className;
    QMetaObject metaObject = ObjectFactory::getObjectType(className);
    QStandardItem* parentItem = _factoryClassTree->invisibleRootItem();
    for (int i=0; i < metaObject.classInfoCount(); i++) {
      QMetaClassInfo info = metaObject.classInfo(i);
      if (info.name() == QStringLiteral("directory")) {
        QString path = info.value();
        QStringList names = path.split(QChar('/'),QString::SkipEmptyParts);
        for (auto name : names) {
          bool found = false;
          for (int i = 0; i < parentItem->rowCount(); i++) {
            if (parentItem->child(i)->text() == name) {
              parentItem = parentItem->child(i);
              found = true;
              break;
            }
          }
          if (!found) {
            QStandardItem* item = new QStandardItem(name);
            parentItem->appendRow(item);
            parentItem = item;
          }
        }
      }
      else if (info.name() == QStringLiteral("class-alias")) {
        label = info.value();
      }
    }
    QStandardItem* item = new QStandardItem(label);
    item->setData(className);
    parentItem->appendRow(item);
  }
}

void QObjectGraphEditorApp::onSceneSelectionChanged() {
  qDebug() << Q_FUNC_INFO;
  QGraphicsScene* scene = ui->objectView->scene();
  QList<QGraphicsItem*> selection = scene->selectedItems();
  if (selection.count() == 1) {
    QGraphicsItem* item = selection.first();
    ObjectGraphNode* object = dynamic_cast<ObjectGraphNode*>(item);
    if (object) {
      ui->propertySheet->setObject(object->object());
      return;
    }
  }
  ui->propertySheet->setObject((QObject*)nullptr);
}

bool QObjectGraphEditorApp::open(const QString& fileName) {
  QFile file(fileName);
  if (file.open(QIODevice::ReadOnly)) {
    QDataStream in(&file);
    _graph->clear();
    if (_graph->read(in)) {
      _graph->setFileName(fileName);
      ui->objectView->onRecenterPressed();
      return true;
    }
    else {
      QMessageBox::information(this, tr("Unable to read model"), file.errorString());
      return false;
    }
  }
  else {
    QMessageBox::information(this, tr("Unable to open input file"), file.errorString());
    return false;
  }
}

bool QObjectGraphEditorApp::save(const QString& fileName) {
  QFile file(fileName);
  if (file.open(QIODevice::WriteOnly)) {
    QDataStream out(&file);
    ui->objectView->scene()->clearSelection();
    if (_graph->write(out)) {
      _graph->setFileName(fileName);
      return true;
    }
    else {
      QMessageBox::information(this, tr("Unable to write model"), file.errorString());
      return false;
    }
  }
  else {
    QMessageBox::information(this, tr("Unable to open output file"), file.errorString());
    return false;
  }
  return false;
}

void QObjectGraphEditorApp::newModel() {
  _graph->clear();
}

void QObjectGraphEditorApp::openModel() {
  QString fileName = QFileDialog::getOpenFileName(this,
                                                  tr("Open QObject Graph"), "",
                                                  tr("QObject Graph (*.qgr);;All Files (*)"));
  if (open(fileName)) {
    QAction* actionOpenRecent = new QAction(fileName);
    connect(actionOpenRecent,&QAction::triggered,this,&QObjectGraphEditorApp::openRecentModel);
    ui->menuRecentModels->insertAction(ui->editMenu->actions().first(),actionOpenRecent);
  }
}

void QObjectGraphEditorApp::openRecentModel(bool) {
  QAction* action = qobject_cast<QAction*>(sender());
  if (action) {
    open(action->text());
  }
}

void QObjectGraphEditorApp::saveModel() {

  if (_graph->fileName().isEmpty()) {
    saveModelAs();
  }
  else {
    save(_graph->fileName());
  }
}

void QObjectGraphEditorApp::saveModelAs() {
  QString fileName = QFileDialog::getSaveFileName(this,
                                                  tr("Save QObject Graph"), "",
                                                  tr("QObject Graph (*.qgr);;All Files (*)"));
  if (save(fileName)) {
    QAction* actionOpenRecent = new QAction(fileName);
    connect(actionOpenRecent,&QAction::triggered,this,&QObjectGraphEditorApp::openRecentModel);
    ui->menuRecentModels->insertAction(ui->editMenu->actions().first(),actionOpenRecent);
  }
}

void QObjectGraphEditorApp::recentModels() {

}

void QObjectGraphEditorApp::closeModel() {

}

void QObjectGraphEditorApp::exitApp() {

}

void QObjectGraphEditorApp::undo() {

}

void QObjectGraphEditorApp::redo() {

}

void QObjectGraphEditorApp::cut() {

}

void QObjectGraphEditorApp::copy() {

}

void QObjectGraphEditorApp::paste() {

}

void QObjectGraphEditorApp::selectAll() {

}

void QObjectGraphEditorApp::toggleFullScreen() {

}

void QObjectGraphEditorApp::about() {

}

void QObjectGraphEditorApp::aboutPlugins() {

}
