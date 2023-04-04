#ifndef DRAWCONTOURS_H
#define DRAWCONTOURS_H

#include <QObject>
#include <QMetaType>
#include <QColor>
#include "AbstractOpenCvObject.h"

#include <opencv2/core.hpp>
#include <vector>

typedef std::vector<std::vector<cv::Point>> Contours;

Q_DECLARE_METATYPE(Contours);

class DrawContours : public AbstractOpenCvObject
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

  void contours(const Contours& contours);

  QVARIANT_PAYLOAD(MatEvent) void image(const QVariant& dstEvent);

  void contourIndex(int index);

  void color(const QColor& color);

  void thickness(int thickness);

  void lineType(int lineType);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

private:

  cv::Mat _image;
  Contours _contours;

  int _contourIndex;
  QColor _color;
  int _thickness;
  int _lineType;

};

#endif // DRAWCONTOURS_H
