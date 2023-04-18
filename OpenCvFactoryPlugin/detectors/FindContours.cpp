#include "detectors/FindContours.h"
#include <QDebug>
#include "ObjectModel.h"

REGISTER_CLASS(FindContours)

FindContours::FindContours(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _nextInput()
  , _mode(RETR_TREE)
  , _method(CHAIN_APPROX_SIMPLE)
  , _offset(0,0)
  , _firstContour(0)
  , _lastContour(255)
  , _stepSize(16)
{
  qRegisterMetaType<Contours>();
}

FindContours::~FindContours() {
  _nextInput = cv::Mat();
}

void FindContours::mode(Mode mode) {
  _mode = mode;
}

void FindContours::method(Method method) {
  _method = method;
}

void FindContours::offset(const QPoint& offset) {
  _offset = offset;
}

void FindContours::firstContour(int firstContour) {
  if (firstContour > 0) {
    _firstContour = firstContour;
    if (_firstContour > _lastContour) _lastContour = _firstContour;
  }
}

void FindContours::lastContour(int lastContour) {
  if (lastContour > 0) {
    _lastContour = lastContour;
    if (_lastContour < _firstContour) _firstContour = _lastContour;
  }
}

void FindContours::stepSize(int stepSize) {
  if (stepSize > 0) {
    _stepSize = stepSize;
  }
}

void FindContours::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
    MatEvent matEvent = qvariant_cast<MatEvent>(variant);

    Contours allContours;

    for (int step = _firstContour; step < _lastContour; step += _stepSize) {
        cv::Mat thresh;
        cv::threshold(matEvent.mat(),thresh,step,255,1);
        Contours contours;
        cv::findContours(thresh,contours,_mode,_method,cv::Point(_offset.x(),_offset.y()));
        allContours.insert(allContours.end(), contours.begin(), contours.end());
    }

    emit src(QVariant::fromValue(MatEvent(matEvent.mat(),matEvent.timestamp())));
    emit contours(allContours);

  }
  else {
    ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
