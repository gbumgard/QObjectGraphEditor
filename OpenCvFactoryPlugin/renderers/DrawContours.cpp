#include "renderers/DrawContours.h"
#include "OpenCvFactoryPlugin.h"
#include <opencv2/imgproc.hpp>
#include <QDebug>

REGISTER_CLASS(DrawContours)

DrawContours::DrawContours(QObject* parent)
  : ThreadedObject(parent)
  , _image()
  , _contours()
  , _contourIndex(-1)
  , _color(Qt::white)
  , _thickness(1)
  , _lineType(8)
{
  start();
}

DrawContours::~DrawContours() {
}

void DrawContours::image(const cv::Mat &mat) {
  UpdateLock lock(this);
  _image = mat;
}

void DrawContours::contours(const std::vector<std::vector<cv::Point> > &contours) {
  UpdateLock lock(this);
  _contours = contours;
}

void DrawContours::contourIndex(int index) {
  UpdateLock lock(this);
  _contourIndex = index;
}

void DrawContours::color(const QColor &color) {
  UpdateLock lock(this);
  _color = color;
}

void DrawContours::thickness(int thickness) {
  UpdateLock lock(this);
  _thickness = thickness;
}

void DrawContours::lineType(int lineType) {
  UpdateLock lock(this);
  _lineType = lineType;
}

void DrawContours::update() {
  cv::Mat currentImage;
  std::vector<std::vector<cv::Point> > currentContours;
  int currentIndex;
  QColor currentColor;
  int currentThickness;
  int currentLineType;
  {
    WaitForUpdateLock lock(this);
    if (_stop) return;
    currentImage = _image.clone();
    _image = cv::Mat();
    if (!currentImage.empty()) {
      currentContours = _contours;
      currentIndex = _contourIndex;
      currentColor = _color;
      currentThickness = _thickness;
      currentLineType = _lineType;
    }
    else {
      return;
    }
  }
  int r,g,b;
  currentColor.getRgb(&r, &g, &b);
  cv::Scalar color(b,g,r);
  cv::drawContours(currentImage,currentContours,currentIndex,color,currentThickness,currentLineType);
  emit out(currentImage);
}
