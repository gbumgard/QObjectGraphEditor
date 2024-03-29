#ifndef APPLYCOLORMAP_H
#define APPLYCOLORMAP_H


#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <opencv2/imgproc.hpp>

class ApplyColorMap : public AbstractOpenCvObject
{

  Q_OBJECT

public:

  enum ColorMap {
    COLORMAP_AUTUMN = cv::COLORMAP_AUTUMN,
    COLORMAP_BONE = cv::COLORMAP_BONE,
    COLORMAP_JET = cv::COLORMAP_JET,
    COLORMAP_WINTER = cv::COLORMAP_WINTER,
    COLORMAP_RAINBOW = cv::COLORMAP_RAINBOW,
    COLORMAP_OCEAN = cv::COLORMAP_OCEAN,
    COLORMAP_SUMMER = cv::COLORMAP_SUMMER,
    COLORMAP_SPRING = cv::COLORMAP_SPRING,
    COLORMAP_COOL = cv::COLORMAP_COOL,
    COLORMAP_HSV = cv::COLORMAP_HSV,
    COLORMAP_PINK = cv::COLORMAP_PINK,
    COLORMAP_HOT = cv::COLORMAP_HOT,
    COLORMAP_PARULA = cv::COLORMAP_PARULA,
    COLORMAP_MAGMA = cv::COLORMAP_MAGMA,
    COLORMAP_INFERNO = cv::COLORMAP_INFERNO,
    COLORMAP_PLASMA = cv::COLORMAP_PLASMA,
    COLORMAP_VIRIDIS = cv::COLORMAP_VIRIDIS,
    COLORMAP_CIVIDIS = cv::COLORMAP_CIVIDIS,
    COLORMAP_TWILIGHT = cv::COLORMAP_TWILIGHT,
    COLORMAP_TWILIGHT_SHIFTED = cv::COLORMAP_TWILIGHT_SHIFTED,
    COLORMAP_TURBO = cv::COLORMAP_TURBO,
    COLORMAP_DEEPGREEN = cv::COLORMAP_DEEPGREEN
  };

  Q_ENUM(ColorMap)

private:

  Q_CLASSINFO("class-alias","Apply Color Map")
  Q_CLASSINFO("directory","OpenCV/Image Processing")

  Q_PROPERTY(ColorMap colorMap READ colorMap WRITE colorMap NOTIFY colorMapChanged)

public:

  Q_INVOKABLE ApplyColorMap(QObject* parent = nullptr);

  ColorMap colorMap() const { return _colorMap; }
  void colorMap(ColorMap colorMap);

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void colorMapChanged(const ColorMap&);

protected:

  void update();

private:

  MatEvent _srcEvent;
  ColorMap _colorMap;
};

#endif // APPLYCOLORMAP_H
