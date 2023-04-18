#ifndef COPYMASKOPERATION_H
#define COPYMASKOPERATION_H

#include <QObject>
#include "AbstractOpenCvObject.h"

#include <opencv2/core.hpp>

class MaskAndCopyOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mask And Copy")
  Q_CLASSINFO("directory","OpenCV/Operations")

public:

  Q_INVOKABLE explicit MaskAndCopyOperation(QObject* parent = nullptr);

  virtual ~MaskAndCopyOperation() {}

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);
  QVARIANT_PAYLOAD(MatEvent) void mask(const QVariant& srcEvent);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& srcEvent);

protected:

  void update();

private:

  cv::Mat _input;
  cv::Mat _mask;
  cv::Mat _output;

};

#endif // COPYMASKOPERATION_H
