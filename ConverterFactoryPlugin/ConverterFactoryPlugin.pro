#-------------------------------------------------
#
# Project created by QtCreator 2018-02-20T17:44:32
#
#-------------------------------------------------

QT       += core gui

TARGET = ConverterFactoryPlugin
TEMPLATE = lib
CONFIG += plugin

DESTDIR = ../plugins

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

INCLUDEPATH += \
  $$PWD/../ObjectModel

DEPENDPATH += \
  $$PWD/../ObjectModel

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    bool/BoolToIntConverter.cpp \
    char/signed/CharToShortConverter.cpp \
    char/signed/CharToIntConverter.cpp \
    char/signed/CharToLongConverter.cpp \
    char/signed/CharToLongLongConverter.cpp \
    char/signed/CharToUnsignedCharConverter.cpp \
    char/signed/CharToUnsignedIntConverter.cpp \
    char/signed/CharToUnsignedLongConverter.cpp \
    char/signed/CharToUnsignedLongLongConverter.cpp \
    char/signed/CharToFloatConverter.cpp \
    char/signed/CharToDoubleConverter.cpp \
    char/signed/CharToLongDoubleConverter.cpp \
    double/DoubleToIntConverter.cpp \
    double/DoubleToLongConverter.cpp \
    int/signed/IntToBoolConverter.cpp \
    int/signed/IntToCharConverter.cpp \
    int/signed/IntToDoubleConverter.cpp \
    int/signed/IntToFloatConverter.cpp \
    int/signed/IntToLongConverter.cpp \
    int/signed/IntToLongDoubleConverter.cpp \
    int/signed/IntToLongLongConverter.cpp \
    int/signed/IntToShortConverter.cpp \
    int/signed/IntToQVariantConverter.cpp \
    int/signed/IntToUnsignedCharConverter.cpp \
    int/signed/IntToUnsignedIntConverter.cpp \
    int/signed/IntToUnsignedLongConverter.cpp \
    int/signed/IntToUnsignedLongLongConverter.cpp

HEADERS += \
    ConverterFactoryPlugin.h \
    bool/BoolToIntConverter.h \
    char/signed/CharToDoubleConverter.h \
    char/signed/CharToFloatConverter.h \
    char/signed/CharToIntConverter.h \
    char/signed/CharToLongConverter.h \
    char/signed/CharToLongDoubleConverter.h \
    char/signed/CharToLongLongConverter.h \
    char/signed/CharToShortConverter.h \
    char/signed/CharToUnsignedCharConverter.h \
    char/signed/CharToUnsignedIntConverter.h \
    char/signed/CharToUnsignedLongConverter.h \
    char/signed/CharToUnsignedLongLongConverter.h \
    double/DoubleToIntConverter.h \
    double/DoubleToLongConverter.h \
    int/signed/IntToBoolConverter.h \
    int/signed/IntToCharConverter.h \
    int/signed/IntToDoubleConverter.h \
    int/signed/IntToFloatConverter.h \
    int/signed/IntToLongConverter.h \
    int/signed/IntToLongDoubleConverter.h \
    int/signed/IntToLongLongConverter.h \
    int/signed/IntToShortConverter.h \
    int/signed/IntToQVariantConverter.h \
    int/signed/IntToUnsignedCharConverter.h \
    int/signed/IntToUnsignedIntConverter.h \
    int/signed/IntToUnsignedLongConverter.h \
    int/signed/IntToUnsignedLongLongConverter.h

DISTFILES += \ 
    ConverterFactoryPlugin.json

unix {
    target.path = ../plugins
    INSTALLS += target
}
