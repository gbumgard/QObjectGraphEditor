#-------------------------------------------------
#
# Project created by QtCreator 2018-03-20T22:12:47
#
#-------------------------------------------------

QT       += sql widgets

TARGET = ObjectViewModel
TEMPLATE = lib

DEFINES += OBJECTVIEWMODEL_LIBRARY

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
    ConnectionPoint.cpp \
    SlotConnectionPoint.cpp \
    MethodMimeData.cpp \
    SlotMimeData.cpp \
    SignalMimeData.cpp \
    SignalConnectionPoint.cpp \
    ObjectGraph.cpp \
    ObjectGraphNode.cpp \
    ObjectGraphEdge.cpp

HEADERS += \
    objectview_global.h \
    ConnectionPoint.h \
    SlotConnectionPoint.h \
    MethodMimeData.h \
    SlotMimeData.h \
    SignalMimeData.h \
    SignalConnectionPoint.h \
    ObjectGraph.h \
    ObjectGraphNode.h \
    ObjectGraphEdge.h


unix {
    target.path = /usr/lib
    INSTALLS += target
}

win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ObjectModel/release/ -lObjectModel
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ObjectModel/debug/ -lObjectModel
else:unix: LIBS += -L$$OUT_PWD/../ObjectModel/ -lObjectModel

INCLUDEPATH += \
  $$PWD/ \
  $$PWD/commands

INCLUDEPATH += $$PWD/../ObjectModel
DEPENDPATH += $$PWD/../ObjectModel


win32:CONFIG(release, debug|release): LIBS += -L$$OUT_PWD/../ObjectGraphUtil/release/ -lObjectGraphUtil
else:win32:CONFIG(debug, debug|release): LIBS += -L$$OUT_PWD/../ObjectGraphUtil/debug/ -lObjectGraphUtil
else:unix: LIBS += -L$$OUT_PWD/../ObjectGraphUtil/ -lObjectGraphUtil

INCLUDEPATH += $$PWD/../ObjectGraphUtil
DEPENDPATH += $$PWD/../ObjectGraphUtil
