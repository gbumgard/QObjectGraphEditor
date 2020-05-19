#-------------------------------------------------
#
# Project created by QtCreator 2018-03-15T15:12:33
#
#-------------------------------------------------

QT       += core widgets gui
QT +=  opengl

TARGET = OpenCvFactoryPlugin
TEMPLATE = lib
CONFIG += plugin

DESTDIR = ../plugins

LIBS += \
  -lopencv_core \
  -lopencv_video \
  -lopencv_videoio \
  -lopencv_imgproc \
  -lopencv_ximgproc\
  -lopencv_imgcodecs \
  -lopencv_highgui \
  -lfreenect \
  #-lfreenect_cv \
  -lfreenect2 \
  -lglut \
  -lpthread \
  -lusb-1.0
#  -lpcl_common \
#  -lpcl_cuda_features \
#  -lpcl_cuda_io \
#  -lpcl_cuda_segmentation

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS \
  DLIB_JPEG_SUPPORT \
  DLIB_USE_BLAS \
  DLIB_USE_LAPACK \
  DLIB_USE_CUDA \
  DLIB_PNG_SUPPORT \
  LAPACK_FORCE_UNDERSCORE

INCLUDEPATH += \
  $$PWD/../ObjectModel

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/local/include/pcl-1.8
INCLUDEPATH += /usr/include/libusb-1.0


DEPENDPATH += \
  $$PWD/../ObjectModel

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    OpenCvFactoryPlugin.cpp \
    common/AbstractOpenCvObject.cpp \
    operations/AddOperation.cpp \
    operations/AddWeightedOperation.cpp \
    transformations/ClipAndNormalize.cpp \
    transformations/ConvertScaleAbs.cpp \
    filters/GaussianBlurFilter.cpp \
    operations/MaskOperation.cpp \
    converters/MatToQImageConvertor.cpp \
    viewers/MatViewer.cpp \
    transformations/MorphologyTransformation.cpp \
    sensors/OpenNiSensor.cpp \
    filters/RunningAverageFilter.cpp \
    detectors/CannyLineDetector.cpp \
    common/ThreadedObject.cpp \
    operations/MirrorMaskOperation.cpp \
    operations/MaskAndCopyOperation.cpp \
    imageprocessing/AnisotropicDiffusion.cpp \
    sensors/KinectV2Sensor.cpp \
    filters/MedianFilter.cpp \
    operations/SubtractOperation.cpp \
    utility/MatQueue.cpp \
    detectors/FindContours.cpp \
    renderers/DrawContours.cpp \
    operations/AbsDifferenceOperation.cpp \
    filters/TemporalMedianFilter.cpp \
    filters/ThresholdFilter.cpp \
    sensors/KinectV1Sensor.cpp \
    imageprocessing/ApplyColorMap.cpp \
    imageprocessing/ApplyTerrainColorMap.cpp \
    utility/MatInfo.cpp \
    filters/KinectV1DepthFilter.cpp \
    utility/RunningStatsUtility.cpp \
    utility/Statistics.cpp \
    filters/StatFilter.cpp \
    filters/DepthFilter.cpp \
    filters/ChangeCommitFilter.cpp \
    filters/MaxFilter.cpp


HEADERS += \
    OpenCvFactoryPlugin.h \
    common/AbstractOpenCvObject.h \
    operations/AddOperation.h \
    operations/AddWeightedOperation.h \
    transformations/ClipAndNormalize.h \
    filters/GaussianBlurFilter.h \
    operations/MaskOperation.h \
    converters/MatToQImageConvertor.h \
    viewers/MatViewer.h \
    transformations/MorphologyTransformation.h \
    transformations/ConvertScaleAbs.h \
    sensors/OpenNiSensor.h \
    filters/RunningAverageFilter.h \
    detectors/CannyLineDetector.h \
    common/ThreadedObject.h \
    operations/MirrorMaskOperation.h \
    operations/MaskAndCopyOperation.h \
    imageprocessing/AnisotropicDiffusion.h \
    sensors/KinectV2Sensor.h \
    filters/MedianFilter.h \
    operations/SubtractOperation.h \
    utility/MatQueue.h \
    detectors/FindContours.h \
    renderers/DrawContours.h \
    operations/AbsDifferenceOperation.h \
    filters/TemporalMedianFilter.h \
    filters/ThresholdFilter.h \
    sensors/KinectV1Sensor.h \
    imageprocessing/ApplyColorMap.h \
    imageprocessing/ApplyTerrainColorMap.h \
    utility/MatInfo.h \
    filters/KinectV1DepthFilter.h \
    utility/RunningStatsUtility.h \
    utility/Statistics.h \
    filters/StatFilter.h \
    filters/DepthFilter.h \
    filters/ChangeCommitFilter.h \
    filters/MaxFilter.h


DISTFILES += OpenCvFactoryPlugin.json 

INCLUDEPATH += \
  $$PWD/../ObjectModel \
  $$PWD/common

INCLUDEPATH += /usr/local/include

DEPENDPATH += \
  $$PWD/../ObjectModel

unix {
    target.path = /usr/lib
    INSTALLS += target
}
