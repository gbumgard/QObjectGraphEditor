#include "utility/ThreadDecoupler.h"
#include "OpenCvFactoryPlugin.h"
#include <QDebug>
#include <QCoreApplication>

REGISTER_CLASS(ThreadDecoupler)

ThreadDecoupler::ThreadDecoupler(QObject* parent)
  : ThreadedObject(parent)
  , _nextInput()
{
  start();
}

ThreadDecoupler::~ThreadDecoupler() {
  _nextInput = cv::Mat();
}

void ThreadDecoupler::in(const cv::Mat& mat) {
  UpdateLock lock(this);
  _nextInput = mat;
  qDebug() << Q_FUNC_INFO;
}

void ThreadDecoupler::update() {

  cv::Mat currentInput;
  {
    WaitForUpdateLock lock(this);
    if (_stop) return;
    currentInput = _nextInput.clone();
    _nextInput.release();
  }
  if (!currentInput.empty()) {
    qDebug() << Q_FUNC_INFO;
    emit out(currentInput);
    QCoreApplication::processEvents();
  }
}
