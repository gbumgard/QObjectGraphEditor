#include "renderers/DrawContours.h"
#include "ObjectModel.h"
#include <opencv2/imgproc.hpp>
#include <QDebug>

REGISTER_CLASS(DrawContours)

DrawContours::DrawContours(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _image()
  , _contours()
  , _contourIndex(-1)
  , _color(Qt::white)
  , _thickness(1)
  , _lineType(8)
{
}

DrawContours::~DrawContours() {
}

void DrawContours::image(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
        MatEvent matEvent = qvariant_cast<MatEvent>(variant);
        _image = matEvent.mat();
    }
    else {
        ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

void DrawContours::contourIndex(int index) {
  _contourIndex = index;
}

void DrawContours::color(const QColor &color) {
  _color = color;
}

void DrawContours::thickness(int thickness) {
  _thickness = thickness;
}

void DrawContours::lineType(int lineType) {
  _lineType = lineType;
}

void DrawContours::contours(const Contours &contours) {
  _contours = contours;
  cv::Mat currentImage;
  Contours currentContours;
  int currentIndex;
  QColor currentColor;
  int currentThickness;
  int currentLineType;
  {
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
  emit out(QVariant::fromValue(MatEvent(currentImage)));
}
