#ifndef APPLYTERRAINCOLORMAP_H
#define APPLYTERRAINCOLORMAP_H

#include "OpenCvFactoryPlugin.h"

#include <QObject>

#include <opencv2/core.hpp>

class ApplyTerrainColorMap : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Apply Terrain Color Map")
  Q_CLASSINFO("directory","OpenCV/Image Processing")

  Q_PROPERTY(bool flip READ flip WRITE flip)
public:

  Q_INVOKABLE ApplyTerrainColorMap(QObject* parent = nullptr);

  virtual ~ApplyTerrainColorMap() {}

  bool flip() const { return _flip; }

signals:

  void dst(const MatEvent& mat);

public slots:

  void src(const MatEvent& mat);

  void flip(bool enable);

private:

  bool _flip;
  cv::Mat _input;

};

#endif // APPLYTERRAINCOLORMAP_H
