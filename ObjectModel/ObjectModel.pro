#-------------------------------------------------
#
# Project created by QtCreator 2018-03-20T14:47:14
#
#-------------------------------------------------

QT       += sql widgets

QT       -= gui

TARGET = ObjectModel
TEMPLATE = lib

DEFINES += OBJECTMODEL_LIBRARY

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
    Connection.cpp \
    ObjectFactory.cpp \
    ObjectModel.cpp \
    SimpleObject.cpp \
    ThreadedObject.cpp

HEADERS += \
  Constraints.h \
  Event.h \
  QVariantEvent.h \
  SimpleObject.h \
  ThreadedObject.h \
  WidgetObject.h \
  objectmodel_global.h \
  ObjectFactory.h \
  Connection.h \
    ObjectModel.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}
