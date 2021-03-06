#-------------------------------------------------
#
# Project created by QtCreator 2018-02-20T17:45:31
#
#-------------------------------------------------

QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QObjectGraphEditorApp
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    main.cpp \
    QObjectGraphEditorApp.cpp \
    PropertySheet.cpp \
    ObjectView.cpp

HEADERS += \
    QObjectGraphEditorApp.h \
    PropertySheet.h \
    ObjectView.h

FORMS += \
    QObjectGraphEditorApp.ui

INCLUDEPATH += \
  $$PWD/../ObjectViewModel \
  $$PWD/../ObjectModel \
  $$PWD/../../QtPropertyBrowser/src

DEPENDPATH += \
  $$PWD/../ObjectViewModel \
  $$PWD/../ObjectModel \
  $$PWD/../../QtPropertyBrowser/src

#QMAKE_POST_LINK += mkdir -f $$OUT_PWD/db

exists($$OUT_PWD/db) {
  message("db directory exists")
}
else {
  QMAKE_POST_LINK += mkdir -f $$OUT_PWD/db
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ObjectViewModel/release/ -lObjectViewModel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ObjectViewModel/debug/ -lObjectViewModel
else:unix: LIBS += -L$$OUT_PWD/../ObjectViewModel/ -lObjectViewModel

INCLUDEPATH += $$PWD/../ObjectGraphView
DEPENDPATH += $$PWD/../ObjectGraphView

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/release/ -lQtPropertyBrowser
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/debug/ -lQtPropertyBrowser
else:unix: LIBS += -L$$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/ -lQtPropertyBrowser

INCLUDEPATH += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src
DEPENDPATH += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/release/libQtPropertyBrowser.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/debug/libQtPropertyBrowser.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/release/QtPropertyBrowser.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/debug/QtPropertyBrowser.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../build-QtPropertyBrowser-Desktop_Qt_5_11_1_GCC_64bit-Default/src/libQtPropertyBrowser.a

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ObjectModel/release/ -lObjectModel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ObjectModel/debug/ -lObjectModel
else:unix: LIBS += -L$$OUT_PWD/../ObjectModel/ -lObjectModel

INCLUDEPATH += $$PWD/../ObjectModel
DEPENDPATH += $$PWD/../ObjectModel

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ObjectGraphUtil/release/ -lObjectGraphUtil
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ObjectGraphUtil/debug/ -lObjectGraphUtil
else:unix: LIBS += -L$$OUT_PWD/../ObjectGraphUtil/ -lObjectGraphUtil

INCLUDEPATH += $$PWD/../ObjectGraphUtil
DEPENDPATH += $$PWD/../ObjectGraphUtil
