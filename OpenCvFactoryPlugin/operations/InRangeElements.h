#ifndef InRange_H
#define InRange_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>
#include <opencv2/core.hpp>

class InRange : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","In Range")
  Q_CLASSINFO("directory","OpenCV/core/Operations On Arrays")

public:

  Q_INVOKABLE explicit InRange(QObject* parent = nullptr);

  virtual ~InRange() {}

public slots:

  void src(const MatEvent& srcEvent);
  void lowerb(const MatEvent& lowerbEvent);
  void upperb(const MatEvent& upperbEvent);

signals:

  void dst(const MatEvent& dstEvent);

protected:

  void update();

private:

  cv::Mat _inputMat;
  cv::Mat _lowerbEvent;
  cv::Mat _upperbEvent;

};

#endif // InRange_H
