#ifndef APPLYTERRAINCOLORMAP_H
#define APPLYTERRAINCOLORMAP_H

#include <QObject>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include "MatEvent.h"
#include <opencv2/core.hpp>

class ApplyTerrainColorMap : public AbstractOpenCvObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Apply Terrain Color Map")
  Q_CLASSINFO("directory","OpenCV/Image Processing")

  Q_PROPERTY(bool flip READ flip WRITE flip NOTIFY flipChanged)

public:

  Q_INVOKABLE ApplyTerrainColorMap(QObject* parent = nullptr);

  virtual ~ApplyTerrainColorMap() {}

  bool flip() const { return _flip; }

signals:

  QVARIANT_PAYLOAD(MatEvent) void dst(const QVariant& dstEvent);

  void flipChanged(bool);

public slots:

  QVARIANT_PAYLOAD(MatEvent) void src(const QVariant& srcEvent);

  void flip(bool enable);

private:

  bool _flip;
  MatEvent _srcEvent;

};

#endif // APPLYTERRAINCOLORMAP_H
