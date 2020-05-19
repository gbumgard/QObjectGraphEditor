#ifndef MATQUEUE_H
#define MATQUEUE_H

#include <QObject>
#include <deque>

#include <opencv2/core.hpp>

class MatQueue : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mat Queue")
  Q_CLASSINFO("directory","OpenCV/Utility")

  Q_PROPERTY(int depth READ depth WRITE depth)

public:

  Q_INVOKABLE explicit MatQueue(QObject* parent = nullptr);

  virtual ~MatQueue() {}

  int depth() const { return _depth; }
  void depth(int depth);

public slots:

  void in(const cv::Mat& mat);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  int _depth;
  std::deque<cv::Mat> _queue;

};

#endif // MATQUEUE_H
