#ifndef MATINFO_H
#define MATINFO_H

#include <QObject>

#include <opencv2/core.hpp>

class MatInfo : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mat Info")
  Q_CLASSINFO("directory","OpenCV/Utility")

public:

  Q_INVOKABLE explicit MatInfo(QObject* parent = nullptr);

  virtual ~MatInfo() {}

public slots:

  void in(const cv::Mat& mat);

signals:

  void cols(int cols);
  void rows(int rows);
  void type(int type);
  void channels(int channels);
  void depth(int depth);
  void min(double min);
  void max(double max);

protected:

};

#endif // MATINFO_H
