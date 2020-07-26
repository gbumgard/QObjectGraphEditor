#ifndef TaggedMaskOperation_H
#define TaggedMaskOperation_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>

#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class ApplyMask : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Apply Mask")
  Q_CLASSINFO("directory","OpenCV/Core/Operations On Arrays")

public:

  Q_INVOKABLE explicit ApplyMask(QObject* parent = nullptr);

  virtual ~ApplyMask() {}

public slots:

  void src(const MatEvent& srcEvent);
  void mask(const MatEvent& maskEvent);

signals:

  void dst(const MatEvent& mat);

protected:

  void update();

private:

  MatEvent _srcEvent;
  MatEvent _maskEvent;

};

#endif // TaggedMaskOperation_H
