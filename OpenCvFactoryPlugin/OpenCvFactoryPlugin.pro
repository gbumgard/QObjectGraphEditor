#-------------------------------------------------
#
# Project created by QtCreator 2018-03-15T15:12:33
#
#-------------------------------------------------

QT       += core widgets gui
QT +=  opengl
QT += datavisualization


TARGET = OpenCvFactoryPlugin
TEMPLATE = lib
CONFIG += plugin

DESTDIR = ../plugins

LIBS += -L/usr/local/lib \
  -lopencv_alphamat        -lopencv_cudawarping    -lopencv_imgcodecs            -lopencv_shape \
  -lopencv_aruco           -lopencv_cudev          -lopencv_img_hash             -lopencv_stereo \
  -lopencv_bgsegm          -lopencv_cvv            -lopencv_imgproc              -lopencv_stitching \
  -lopencv_bioinspired     -lopencv_datasets       -lopencv_intensity_transform  -lopencv_structured_light \
  -lopencv_calib3d         -lopencv_dnn_objdetect  -lopencv_line_descriptor      -lopencv_superres \
  -lopencv_ccalib          -lopencv_dnn            -lopencv_ml                   -lopencv_surface_matching \
  -lopencv_core            -lopencv_dnn_superres   -lopencv_objdetect            -lopencv_text \
  -lopencv_cudaarithm      -lopencv_dpm            -lopencv_optflow              -lopencv_tracking \
  -lopencv_cudabgsegm      -lopencv_face           -lopencv_phase_unwrapping     -lopencv_videoio \
  -lopencv_cudacodec       -lopencv_features2d     -lopencv_photo                -lopencv_video \
  -lopencv_cudafeatures2d  -lopencv_flann          -lopencv_plot                 -lopencv_videostab \
  -lopencv_cudafilters     -lopencv_freetype       -lopencv_quality              -lopencv_viz \
  -lopencv_cudaimgproc     -lopencv_fuzzy          -lopencv_rapid                -lopencv_world \
  -lopencv_cudalegacy      -lopencv_gapi           -lopencv_reg                  -lopencv_xfeatures2d \
  -lopencv_cudaobjdetect   -lopencv_hdf            -lopencv_rgbd                 -lopencv_ximgproc \
  -lopencv_cudaoptflow     -lopencv_hfs            -lopencv_saliency             -lopencv_xobjdetect \
  -lopencv_cudastereo      -lopencv_highgui        -lopencv_sfm                  -lopencv_xphoto \
  -lfreenect \
  -lfreenect2 \
  -lglut \
  -lpthread \
  -lusb-1.0
#  -lpcl_common \
#  -lpcl_cuda_features \
#  -lpcl_cuda_io \
#  -lpcl_cuda_segmentation
#-lfreenect_cv \

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
INCLUDEPATH += /usr/local/include/opencv4

RESOURCES += surface.qrc

DEPENDPATH += \
  $$PWD/../ObjectModel

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    OpenCvFactoryPlugin.cpp \
    common/AbstractOpenCvObject.cpp \
    filters/BilateralFilter.cpp \
    filters/DepthFilterV2.cpp \
    filters/InRangeFilter.cpp \
    filters/MedianFilter3D.cpp \
    filters/RunningStatistics.cpp \
    filters/ThresholdFilter.cpp \
    imageprocessing/BackgroundSubtractionMask.cpp \
    imageprocessing/ConvertToGrayScale.cpp \
    operations/AddOperation.cpp \
    operations/AddWeightedOperation.cpp \
    operations/BitwiseNotOperation.cpp \
    operations/BitwiseOrOperation.cpp \
    operations/InRangeOperation.cpp \
    operations/MaskOperation.cpp \
    operations/SetToScalarOperation.cpp \
    transformations/ClipAndNormalize.cpp \
    transformations/ConvertScaleAbs.cpp \
    filters/GaussianBlurFilter.cpp \
    converters/MatToQImageConvertor.cpp \
    utility/MatInfo.cpp \
    utility/ThreadDecoupler.cpp \
    viewers/Mat3DViewer.cpp \
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
    sensors/KinectV1Sensor.cpp \
    imageprocessing/ApplyColorMap.cpp \
    imageprocessing/ApplyTerrainColorMap.cpp \
    filters/KinectV1DepthFilter.cpp \
    utility/RunningStatsUtility.cpp \
    utility/Statistics.cpp \
    filters/StatFilter.cpp \
    filters/DepthFilter.cpp \
    filters/ChangeCommitFilter.cpp \
    filters/MaxFilter.cpp \
    viewers/MatViewer.cpp


HEADERS += \
    OpenCvFactoryPlugin.h \
    common/AbstractOpenCvObject.h \
    filters/BilateralFilter.h \
    filters/DepthFilterV2.h \
    filters/InRangeFilter.h \
    filters/MedianFilter3D.h \
    filters/RunningStatistics.h \
    filters/ThresholdFilter.h \
    imageprocessing/BackgroundSubtractionMask.h \
    imageprocessing/ConvertToGrayScale.h \
    operations/AddOperation.h \
    operations/AddWeightedOperation.h \
    operations/BitwiseNotOperation.h \
    operations/BitwiseOrOperation.h \
    operations/InRangeOperation.h \
    operations/MaskOperation.h \
    operations/SetToScalarOperation.h \
    transformations/ClipAndNormalize.h \
    filters/GaussianBlurFilter.h \
    converters/MatToQImageConvertor.h \
    utility/MatInfo.h \
    utility/ThreadDecoupler.h \
    viewers/Mat3DViewer.h \
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
    sensors/KinectV1Sensor.h \
    imageprocessing/ApplyColorMap.h \
    imageprocessing/ApplyTerrainColorMap.h \
    filters/KinectV1DepthFilter.h \
    utility/RunningStatsUtility.h \
    utility/Statistics.h \
    filters/StatFilter.h \
    filters/DepthFilter.h \
    filters/ChangeCommitFilter.h \
    filters/MaxFilter.h \
    viewers/MatViewer.h


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
