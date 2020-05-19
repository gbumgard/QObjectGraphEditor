#include "QObjectGraphEditorApp.h"

#include <QApplication>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  QObjectGraphEditorApp w;
  w.show();

  return a.exec();
}
