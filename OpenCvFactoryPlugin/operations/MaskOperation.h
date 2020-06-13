#ifndef TaggedMaskOperation_H
#define TaggedMaskOperation_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>

#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MaskOperation : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mask Operation")
  Q_CLASSINFO("directory","OpenCV/Operations")

public:

  Q_INVOKABLE explicit MaskOperation(QObject* parent = nullptr);

  virtual ~MaskOperation() {}

public slots:

  void in(const TaggedMat& taggedMat);
  void mask(const TaggedMat& taggedMat);

signals:

  void out(const TaggedMat& mat);

protected:

  void update();

private:

  TaggedMat _input;
  TaggedMat _mask;

};

#endif // TaggedMaskOperation_H
