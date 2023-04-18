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
# -lopencv_world \
LIBS += -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu \
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
  -lopencv_cudaimgproc     -lopencv_fuzzy          -lopencv_rapid                \
  -lopencv_cudalegacy      -lopencv_gapi           -lopencv_reg                  -lopencv_xfeatures2d \
  -lopencv_cudaobjdetect   -lopencv_hdf            -lopencv_rgbd                 -lopencv_ximgproc \
  -lopencv_cudaoptflow     -lopencv_hfs            -lopencv_saliency             -lopencv_xobjdetect \
  -lopencv_cudastereo      -lopencv_highgui        -lopencv_sfm                  -lopencv_xphoto \
  -lfreenect \
  -lfreenect2 \
  -lglut \
  -lpthread \
  -lusb-1.0 \
  -lpcl_apps              -lpcl_common              -lpcl_features                -lpcl_filters \
  -lpcl_io                -lpcl_io_ply              -lpcl_kdtree                  -lpcl_keypoints \
  -lpcl_ml                -lpcl_octree              -lpcl_outofcore               -lpcl_people \
  -lpcl_recognition       -lpcl_registration        -lpcl_sample_consensus        -lpcl_search \
  -lpcl_segmentation      -lpcl_stereo              -lpcl_surface                 -lpcl_tracking \
  -lpcl_visualization \

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

INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/pcl-1.10
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
  Sandbox.cpp \
  common/AbstractOpenCvObject.cpp \
  detectors/CannyLineDetector.cpp \
  detectors/FindContours.cpp \
  filters/InRangeFilter.cpp \
  filters/MedianBlurFilter.cpp \
  filters/MedianFilter3D.cpp \
  filters/RunningStatistics.cpp \
  filters/SandboxFilter.cpp \
  filters/TemporalMedianFilter.cpp \
  filters/TemporalMedianFilterGpu.cpp \
  filters/ThresholdFilter.cpp \
  filters/StatFilter.cpp \
  filters/BilateralFilter.cpp \
  filters/GaussianBlurFilter.cpp \
  imageprocessing/BackgroundSubtractorCNT.cpp \
  imageprocessing/BackgroundSubtractorKNN.cpp \
  imageprocessing/MatCompare.cpp \
  imageprocessing/AccumulateChanges.cpp \
  imageprocessing/motionanalysis/AccumulateWeighted.cpp \
  imageprocessing/ApplyColorMap.cpp \
  imageprocessing/ApplyTerrainColorMap.cpp \
  operations/AbsDifference.cpp \
  operations/AddWeightedOperation.cpp \
  operations/MaskAndCopyOperation.cpp \
  renderers/DrawContours.cpp \
  sensors/KinectV2SensorAsync.cpp \
  transformations/ClipAndNormalize.cpp \
  transformations/ConvertScaleAbs.cpp \
  transformations/MorphologyTransformation.cpp \
  viewers/MatViewer.cpp \
  viewers/HeightMapViewer.cpp \


#sensors/KinectV2Sensor.cpp \

HEADERS += \
  OpenCvFactoryPlugin.h \
  Sandbox.h \
  common/AbstractOpenCvObject.h \
  common/MatEvent.h \
  common/ScalarEvent.h \
  detectors/CannyLineDetector.h \
  detectors/FindContours.h \
  filters/InRangeFilter.h \
  filters/MedianBlurFilter.h \
  filters/MedianFilter3D.h \
  filters/RunningStatistics.h \
  filters/SandboxFilter.h \
  filters/TemporalMedianFilter.h \
  filters/TemporalMedianFilterGpu.h \
  filters/ThresholdFilter.h \
  filters/StatFilter.h \
  filters/BilateralFilter.h \
  filters/GaussianBlurFilter.h \
  imageprocessing/AccumulateChanges.h \
  imageprocessing/BackgroundSubtractorCNT.h \
  imageprocessing/BackgroundSubtractorKNN.h \
  imageprocessing/MatCompare.h \
  imageprocessing/ApplyColorMap.h \
  imageprocessing/ApplyTerrainColorMap.h \
  imageprocessing/motionanalysis/AccumulateWeighted.h \
  operations/AbsDifference.h \
  operations/AddWeightedOperation.h \
  operations/MaskAndCopyOperation.h \
  renderers/DrawContours.h \
  sensors/KinectV2SensorAsync.h \
  transformations/ClipAndNormalize.h \
  transformations/ConvertScaleAbs.h \
  transformations/MorphologyTransformation.h \
  viewers/MatViewer.h \
  viewers/HeightMapViewer.h \

#sensors/KinectV2Sensor.h \

#SOURCES += \
#    OpenCvFactoryPlugin.cpp \
#    common/AbstractOpenCvObject.cpp \
#    common/MatFingerprint.cpp \
#    filters/DepthFilterV2.cpp \
#    filters/InRangeFilter.cpp \
#    filters/MedianBlurFilter.cpp \
#    filters/MedianFilter3D.cpp \
#    filters/RunningStatistics.cpp \
#    filters/ThresholdFilter.cpp \
#    imageprocessing/BackgroundSubtractionMask.cpp \
#    imageprocessing/ConvertToGrayScale.cpp \
#    imageprocessing/motionanalysis/AccumulateWeighted.cpp \
#    operations/AbsDifference.cpp \
#    operations/AddArray.cpp \
#    operations/AddWeightedOperation.cpp \
#    operations/ApplyMask.cpp \
#    operations/BitwiseAnd.cpp \
#    operations/BitwiseNot.cpp \
#    operations/BitwiseOr.cpp \
#    operations/InRange.cpp \
#    operations/SetToScalarOperation.cpp \
#    transformations/ClipAndNormalize.cpp \
#    transformations/ConvertScaleAbs.cpp \
#    converters/MatToQImageConvertor.cpp \
#    utility/MatInfo.cpp \
#    utility/ThreadDecoupler.cpp \
#    transformations/MorphologyTransformation.cpp \
#    sensors/OpenNiSensor.cpp \
#    common/ThreadedObject.cpp \
#    operations/MirrorMaskOperation.cpp \
#    imageprocessing/AnisotropicDiffusion.cpp \
#    sensors/KinectV2Sensor.cpp \
#    operations/SubtractOperation.cpp \
#    utility/MatQueue.cpp \
#    filters/TemporalMedianFilter.cpp \
#    sensors/KinectV1Sensor.cpp \
#    imageprocessing/ApplyColorMap.cpp \
#    imageprocessing/ApplyTerrainColorMap.cpp \
#    filters/KinectV1DepthFilter.cpp \
#    utility/RunningStatsUtility.cpp \
#    utility/Statistics.cpp \
#    filters/StatFilter.cpp \
#    filters/DepthFilter.cpp \
#    filters/ChangeCommitFilter.cpp \
#    filters/MaxFilter.cpp \
#    viewers/HeightMapViewer.cpp \
#    viewers/MatViewer.cpp


#HEADERS += \
#    OpenCvFactoryPlugin.h \
#    common/AbstractOpenCvObject.h \
#    common/MatEvent.h \
#    common/MatFingerprint.h \
#    common/ScalarEvent.h \
#    common/constraints.h \
#    filters/BilateralFilter.h \
#    filters/DepthFilterV2.h \
#    filters/InRangeFilter.h \
#    filters/MedianBlurFilter.h \
#    filters/MedianFilter3D.h \
#    filters/RunningStatistics.h \
#    filters/ThresholdFilter.h \
#    imageprocessing/BackgroundSubtractionMask.h \
#    imageprocessing/ConvertToGrayScale.h \
#    imageprocessing/motionanalysis/AccumulateWeighted.h \
#    operations/AbsDifference.h \
#    operations/AddArray.h \
#    operations/AddWeightedOperation.h \
#    operations/ApplyMask.h \
#    operations/BitwiseAnd.h \
#    operations/BitwiseNot.h \
#    operations/BitwiseOr.h \
#    operations/InRange.h \
#    operations/InRangeElements.h \
#    operations/SetToScalarOperation.h \
#    transformations/ClipAndNormalize.h \
#    filters/GaussianBlurFilter.h \
#    converters/MatToQImageConvertor.h \
#    utility/MatInfo.h \
#    utility/ThreadDecoupler.h \
#    transformations/MorphologyTransformation.h \
#    transformations/ConvertScaleAbs.h \
#    sensors/OpenNiSensor.h \
#    detectors/CannyLineDetector.h \
#    common/ThreadedObject.h \
#    operations/MirrorMaskOperation.h \
#    operations/MaskAndCopyOperation.h \
#    imageprocessing/AnisotropicDiffusion.h \
#    sensors/KinectV2Sensor.h \
#    operations/SubtractOperation.h \
#    utility/MatQueue.h \
#    detectors/FindContours.h \
#    renderers/DrawContours.h \
#    filters/TemporalMedianFilter.h \
#    sensors/KinectV1Sensor.h \
#    imageprocessing/ApplyColorMap.h \
#    imageprocessing/ApplyTerrainColorMap.h \
#    filters/KinectV1DepthFilter.h \
#    utility/RunningStatsUtility.h \
#    utility/Statistics.h \
#    filters/StatFilter.h \
#    filters/DepthFilter.h \
#    filters/ChangeCommitFilter.h \
#    filters/MaxFilter.h \
#    viewers/HeightMapViewer.h \
#    viewers/MatViewer.h


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
