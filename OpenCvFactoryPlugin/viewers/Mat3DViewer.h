#ifndef Mat3DViewer_H
#define Mat3DViewer_H

#include <QWidget>
#include <QGridLayout>
#include <QMessageBox>
#include <QPainter>
#include <QScreen>
#include <Q3DSurface>
#include <QSurfaceDataProxy>
#include <QHeightMapSurfaceDataProxy>
#include <QSurface3DSeries>

#include <opencv2/core.hpp>

using namespace QtDataVisualization;

class Mat3DViewer : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Height Map Viewer")
  Q_CLASSINFO("directory","OpenCV/Output")

  Q_PROPERTY(QString caption READ caption WRITE caption)
  Q_PROPERTY(bool flipHorizontalGrid READ flipHorizontalGrid WRITE flipHorizontalGrid)
  Q_PROPERTY(bool flatShadingEnabled READ flatShadingEnabled WRITE flatShadingEnabled)
  Q_PROPERTY(DrawMode drawMode READ drawMode WRITE drawMode)
  Q_PROPERTY(bool enableTexture READ enableTexture WRITE enableTexture)

public:

  enum DrawMode {
    DRAW_WIREFRAME = QSurface3DSeries::DrawWireframe,
    DRAW_SURFACE = QSurface3DSeries::DrawSurface,
    DRAW_BOTH = QSurface3DSeries::DrawSurfaceAndWireframe
  };

  Q_ENUM(DrawMode)

  Q_INVOKABLE explicit Mat3DViewer(QObject* parent = nullptr);

  virtual ~Mat3DViewer();

  QString caption() const {
    return QObject::objectName();
  }

  void caption(const QString& caption) {
    QObject::setObjectName(caption);
  }

  bool flipHorizontalGrid() const { return _graph->flipHorizontalGrid(); }
  void flipHorizontalGrid(bool flipHorizontalGrid) { _graph->setFlipHorizontalGrid(flipHorizontalGrid); }

  bool flatShadingEnabled() const { return _heightMapSeries->isFlatShadingEnabled(); }
  void flatShadingEnabled(bool flatShadingEnabled) { _heightMapSeries->setFlatShadingEnabled(flatShadingEnabled); }

  void drawMode(DrawMode drawMode) { _heightMapSeries->setDrawMode((QSurface3DSeries::DrawFlag)drawMode); }
  DrawMode drawMode() const { return (DrawMode)((int)_heightMapSeries->drawMode()); }

  bool enableTexture() const { return _enableTexture; }
  void enableTexture(bool enableTexture) {
    _enableTexture = enableTexture;
    _heightMapSeries->setTexture(_enableTexture ? _texture : QImage());
  }

  QImage heightMap() const { return _heightMap; }
  QImage texture() const { return _texture; }

public slots:

  void map(const cv::Mat& image);
  void texture(const cv::Mat& image);

private:

  QImage _heightMap;
  cv::Size _imageSize;
  QImage _texture;
  bool _enableTexture;

  Q3DSurface* _graph;
  QHeightMapSurfaceDataProxy* _heightMapProxy;
  QSurface3DSeries* _heightMapSeries;

};

#endif // Mat3DViewer_H
