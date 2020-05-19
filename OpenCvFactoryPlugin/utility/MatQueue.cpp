#include "utility/MatQueue.h"
#include "OpenCvFactoryPlugin.h"

REGISTER_CLASS(MatQueue)

MatQueue::MatQueue(QObject* parent)
  : QObject(parent)
  , _depth(1)
{
}

void MatQueue::depth(int depth) {
  if (depth >= 0) {
    _depth = depth;
    while(_queue.size() > (unsigned)_depth) {
      emit out(_queue.front());
      _queue.pop_front();
    }
  }
}

void MatQueue::in(const cv::Mat &mat) {
  if (_depth > 0) {
    _queue.push_back(mat.clone());
    while(_queue.size() > (unsigned)_depth) {
      emit out(_queue.front());
      _queue.pop_front();
    }
  }
  else {
    emit out(mat);
  }
}
