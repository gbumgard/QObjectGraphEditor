#ifndef APPLYCOLORMAP_H
#define APPLYCOLORMAP_H

#include <QObject>
#include <opencv2/imgproc.hpp>

class ApplyColorMap : public QObject
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
    COLORMAP_PARULA = cv::COLORMAP_PARULA
  };

  Q_ENUM(ColorMap)

private:

  Q_CLASSINFO("class-alias","Apply Color Map")
  Q_CLASSINFO("directory","OpenCV/Image Processing")

  Q_PROPERTY(ColorMap colorMap READ colorMap WRITE colorMap)

public:

  Q_INVOKABLE ApplyColorMap(QObject* parent = nullptr);

  ColorMap colorMap() const { return _colorMap; }
  void colorMap(ColorMap colorMap);

signals:

  void out(const cv::Mat& mat);

public slots:

  void in(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _input;
  ColorMap _colorMap;
};

#endif // APPLYCOLORMAP_H
