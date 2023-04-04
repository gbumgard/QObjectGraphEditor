#include "HeightMapViewer.h"
#include "ObjectModel.h"

#include <QDebug>
#include <QGridLayout>
#include <QMessageBox>
#include <QScreen>
#include <QCheckBox>

REGISTER_CLASS(HeightMapViewer)

inline QImage mat_to_qimage_ref(cv::Mat &mat, QImage::Format format)
{
  return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), format);
}

inline cv::Mat qimage_to_mat_ref(QImage &img, int format)
{
  return cv::Mat(img.height(), img.width(), format, img.bits(), img.bytesPerLine());
}

/**
 *@brief make Qimage and cv::Mat share the same buffer, the resource
 * of the cv::Mat must not deleted before the QImage finish
 * the jobs.
 *
 *@param mat : input mat
 *@param swap : true : swap BGR to RGB; false, do nothing
 */
inline QImage mat_to_qimage_ref(cv::Mat &mat, bool swap)
{
  if (!mat.empty()) {

    switch (mat.type()) {
      case CV_8UC3:
        if (swap) {
          return mat_to_qimage_ref(mat, QImage::Format_RGB888).rgbSwapped();
        }
        else {
          return mat_to_qimage_ref(mat, QImage::Format_RGB888);
        }
      case CV_8UC1:
        return mat_to_qimage_ref(mat, QImage::Format_Grayscale8);

      case CV_8UC4:
        return mat_to_qimage_ref(mat, QImage::Format_ARGB32);
    }
  }

  return {};
}

/**
 *@brief copy cv::Mat into QImage
 *
 *@param mat : input mat
 *@param swap : true : swap BGR to RGB; false, do nothing
 */
inline QImage mat_to_qimage_cpy(cv::Mat const &mat, bool swap)
{
  return mat_to_qimage_ref(const_cast<cv::Mat&>(mat), swap).copy();
}

HeightMapViewer::HeightMapViewer(QObject *parent)
  : AbstractOpenCvObject(parent)
  , _heightMap(":/maps/mountain")
  , _imageSize(_heightMap.width(),_heightMap.height())
  , _texture()
  , _enableTexture(false)
{

  _graph = new Q3DSurface();
  _graph->setFlags(_graph->flags() ^ Qt::FramelessWindowHint);

  if (!_graph->hasContext()) {
      QMessageBox msgBox;
      msgBox.setText("Couldn't initialize the OpenGL context.");
      msgBox.exec();
      return;
  }

  QSize screenSize = _graph->screen()->size();
  _graph->setMinimumSize(QSize(screenSize.width() / 4, screenSize.height() / 3.2));
  _graph->setMinimumSize(QSize(screenSize.width() / 4, screenSize.height() / 3.2));
  _graph->setMaximumSize(screenSize);

  _graph->setTitle(QStringLiteral("Height Map Renderer"));

  _graph->setAxisX(new QValue3DAxis);
  _graph->axisX()->setLabelFormat("%.1f");
  _graph->axisX()->setRange(0, 1);
  _graph->axisX()->setTitle(QStringLiteral(""));

  _graph->setAxisY(new QValue3DAxis);
  _graph->axisY()->setRange(0, 255);
  _graph->axisY()->setTitle(QStringLiteral(""));
  _graph->axisY()->setAutoAdjustRange(false);

  _graph->setAxisZ(new QValue3DAxis);
  _graph->axisZ()->setLabelFormat("%.1f");
  _graph->axisZ()->setRange(0, 1);
  _graph->axisZ()->setTitle(QStringLiteral(""));

  _heightMapProxy = new QHeightMapSurfaceDataProxy();
  _heightMapProxy->setHeightMap(_heightMap);
  _heightMapSeries = new QSurface3DSeries(_heightMapProxy);
  _heightMapSeries->setItemLabelFormat(QStringLiteral("(@xLabel, @zLabel): @yLabel"));
  _heightMapProxy->setValueRanges(0.0f, 1.0f, 0.0f, 1.0f);

  _heightMapSeries->setDrawMode(QSurface3DSeries::DrawSurface);
  _heightMapSeries->setFlatShadingEnabled(false);

  _graph->activeTheme()->setType(Q3DTheme::ThemeEbony);//Q3DTheme::Theme(2));
  _graph->activeTheme()->setAmbientLightStrength(1.0);
  _graph->activeTheme()->setBackgroundEnabled(true);
  _graph->activeTheme()->setBackgroundColor(QColor(0,0,0));

  _graph->setSelectionMode(QAbstract3DGraph::SelectionNone);
  _graph->setShadowQuality(QAbstract3DGraph::ShadowQualityNone);


  _graph->addSeries(_heightMapSeries);

  QLinearGradient gr;
  QGradient preset(QGradient::MorpheusDen);
  QGradientStops stops = preset.stops();
  qDebug() << stops;
  gr.setStops(stops);

  _graph->seriesList().at(0)->setBaseGradient(gr);
  _graph->seriesList().at(0)->setColorStyle(Q3DTheme::ColorStyleRangeGradient);
  _graph->show();

}

HeightMapViewer::~HeightMapViewer() {
  _graph->close();
  delete _graph;
}

void HeightMapViewer::map(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);
      cv::Mat heightMat;
      matEvent.mat().convertTo(heightMat,CV_8UC1);
      if (_flipZAxis) {
        heightMat = 255 - heightMat;
      }

      _heightMap = mat_to_qimage_cpy(heightMat,true);

      if (matEvent.mat().size() != _imageSize) {
        _imageSize = matEvent.mat().size();
        _graph->axisX()->setRange(0, _imageSize.width);
        _graph->axisZ()->setRange(0, _imageSize.height);
        _heightMapProxy->setValueRanges(0, _imageSize.width, 0, _imageSize.height);
        _heightMapProxy->setAutoScaleY(false);
        _heightMapProxy->setMinYValue(0.0);
        _heightMapProxy->setMaxYValue(255.0);
      }
      _heightMapProxy->setHeightMap(_heightMap);
    }
    else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}

void HeightMapViewer::texture(const QVariant &variant) {
    if (variant.userType() == MatEvent::userType()) {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);
      if (_enableTexture) {
        _texture = mat_to_qimage_cpy(matEvent.mat(),true);
        _heightMapSeries->setTexture(_texture);
      }
    }
    else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
    }
}
