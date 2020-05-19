#include "detectors/FindContours.h"
#include "OpenCvFactoryPlugin.h"
#include <QDebug>

REGISTER_CLASS(FindContours)

FindContours::FindContours(QObject* parent)
  : ThreadedObject(parent)
  , _nextInput()
  , _mode(RETR_TREE)
  , _method(CHAIN_APPROX_SIMPLE)
  , _offset(0,0)
  , _firstContour(0)
  , _lastContour(255)
  , _stepSize(1)
{
  qRegisterMetaType<std::vector<std::vector<cv::Point>>>();
  start();
}

FindContours::~FindContours() {
  _nextInput = cv::Mat();
}

void FindContours::in(const cv::Mat& mat) {
  UpdateLock lock(this);
  _nextInput = mat;
}

void FindContours::mode(Mode mode) {
  UpdateLock lock(this);
  _mode = mode;
}

void FindContours::method(Method method) {
  UpdateLock lock(this);
  _method = method;
}

void FindContours::offset(const QPoint& offset) {
  UpdateLock lock(this);
  _offset = offset;
}

void FindContours::firstContour(int firstContour) {
  UpdateLock lock(this);
  if (firstContour > 0) {
    _firstContour = firstContour;
    if (_firstContour > _lastContour) _lastContour = _firstContour;
  }
}

void FindContours::lastContour(int lastContour) {
  UpdateLock lock(this);
  if (lastContour > 0) {
    _lastContour = lastContour;
    if (_lastContour < _firstContour) _firstContour = _lastContour;
  }
}

void FindContours::stepSize(int stepSize) {
  UpdateLock lock(this);
  if (stepSize > 0) {
    _stepSize = stepSize;
  }
}

void FindContours::update() {

  cv::Mat currentInput;
  Mode currentMode;
  Method currentMethod;
  QPoint currentOffset;
  int currentFirstContour;
  int currentLastContour;
  int currentStepSize;
  {
    WaitForUpdateLock lock(this);
    if (_stop) return;
    currentInput = _nextInput;
    _nextInput = cv::Mat();
    if (!currentInput.empty()) {
      currentMode = _mode;
      currentMethod = _method;
      currentOffset = _offset;
      currentFirstContour = _firstContour;
      currentLastContour = _lastContour;
      currentStepSize = _stepSize;
    }
    else {
      return;
    }
  }

  std::vector<std::vector<cv::Point> > allContours;

  for (int step = currentFirstContour; step < currentLastContour; step += currentStepSize) {
    cv::Mat thresh;
    cv::threshold(currentInput,thresh,step,255,1);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresh,contours,currentMode,currentMethod,cv::Point(currentOffset.x(),currentOffset.y()));
    allContours.insert(allContours.end(), contours.begin(), contours.end());
  }

  emit source(currentInput);
  emit contours(allContours);
}
