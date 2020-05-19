#ifndef DRAWCONTOURS_H
#define DRAWCONTOURS_H

#include <QObject>
#include <QMetaType>
#include <QColor>
#include "ThreadedObject.h"

#include <opencv2/core.hpp>
#include <vector>

Q_DECLARE_METATYPE(std::vector<std::vector<cv::Point>>)

class DrawContours : public ThreadedObject
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Draw Contours")
  Q_CLASSINFO("directory","OpenCV/Renderers")

  Q_PROPERTY(int contourIndex READ contourIndex WRITE contourIndex)
  Q_PROPERTY(QColor color READ color WRITE color)
  Q_PROPERTY(int thickness READ thickness WRITE thickness)
  Q_PROPERTY(int lineType READ lineType WRITE lineType)

public:

  Q_INVOKABLE explicit DrawContours(QObject *parent = nullptr);

  ~DrawContours();

  int contourIndex() const { return _contourIndex; }

  QColor color() const { return _color; }

  int thickness() const { return _thickness; }

  int lineType() const { return _lineType; }

public slots:

  void contours(const std::vector<std::vector<cv::Point>>& contours);

  void image(const cv::Mat& mat);

  void contourIndex(int index);

  void color(const QColor& color);

  void thickness(int thickness);

  void lineType(int lineType);

signals:

  void out(const cv::Mat& mat);

protected:

  void update() override;

private:

  cv::Mat _image;
  std::vector<std::vector<cv::Point>> _contours;

  int _contourIndex;
  QColor _color;
  int _thickness;
  int _lineType;

};

#endif // DRAWCONTOURS_H
