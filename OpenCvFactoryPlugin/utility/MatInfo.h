#ifndef TaggedMatInfo_H
#define TaggedMatInfo_H

#include "OpenCvFactoryPlugin.h"
#include <QObject>

#include <opencv2/core.hpp>

class MatInfo : public QObject
{

  Q_OBJECT

  Q_CLASSINFO("class-alias","Mat Info")
  Q_CLASSINFO("directory","OpenCV/Utility")

  Q_PROPERTY(int seq READ seq)
  Q_PROPERTY(int rows READ rows)
  Q_PROPERTY(int cols READ cols)
  Q_PROPERTY(int channels READ channels)
  Q_PROPERTY(MatType type READ type)
  Q_PROPERTY(MatDepth depth READ depth)
  Q_PROPERTY(double min READ min)
  Q_PROPERTY(double max READ max)

public:

  enum MatDepth {
    Depth8U = CV_8U,
    Depth8S = CV_8S,
    Depth16U = CV_16U,
    Depth1S6 = CV_16S,
    Depth16 = CV_32S,
    Depth32F = CV_32F,
    Depth64F = CV_64F,
    Depth16F = CV_16F
  };

  Q_ENUM(MatDepth)

  enum MatType {
      Type8UC1 = CV_8UC1,
      Type8UC2 = CV_8UC2,
      Type8UC3 = CV_8UC3,
      Type8UC4 = CV_8UC4,
      Type8SC1 = CV_8SC1,
      Type8SC2 = CV_8SC2,
      Type8SC3 = CV_8SC3,
      Type8SC4 = CV_8SC4,
      Type16UC1 = CV_16UC1,
      Type16UC2 = CV_16UC2,
      Type16UC3 = CV_16UC3,
      Type16UC4 = CV_16UC4,
      Type16SC1 = CV_16SC1,
      Type16SC2 = CV_16SC2,
      Type16SC3 = CV_16SC3,
      Type16SC4 = CV_16SC4,
      Type32SC1 = CV_32SC1,
      Type32SC2 = CV_32SC2,
      Type32SC3 = CV_32SC3,
      Type32SC4 = CV_32SC4,
      Type32FC1 = CV_32FC1,
      Type32FC2 = CV_32FC2,
      Type32FC3 = CV_32FC3,
      Type32FC4 = CV_32FC4,
      Type64FC1 = CV_64FC1,
      Type64FC2 = CV_64FC2,
      Type64FC3 = CV_64FC3,
      Type64FC4 = CV_64FC4,
      Type16FC1 = CV_16FC1,
      Type16FC2 = CV_16FC2,
      Type16FC3 = CV_16FC3,
      Type16FC4 = CV_16FC4
  };

  Q_ENUM(MatType)

  Q_INVOKABLE explicit MatInfo(QObject* parent = nullptr);

  virtual ~MatInfo() {}

  int seq() const { return _seq; }
  int cols() const { return _cols; }
  int rows() const { return _rows; }
  int channels() const { return _channels; }
  MatType type() const { return _type; };
  MatDepth depth() const { return _depth; }
  double min() const { return _min; }
  double max() const { return _max; }

  void setSeq(int newSeq) {
    _seq = newSeq;
    emit seq(_seq);
  }

  void setCols(int newCols) {
    _cols = newCols;
    emit cols(_cols);
  }

  void setRows(int newRows) {
    _rows = newRows;
    emit rows(_rows);
  }

  void setChannels(int newChannels) {
    _channels = newChannels;
    emit channels((int)_channels);
  }

  void setType(MatType newType) {
    _type = newType;
    emit type((int)_type);
  }

  void setDepth(MatDepth newDepth) {
    _depth = newDepth;
    emit depth((int)_depth);
  }

  void setMin(double newMin) {
    _min = newMin;
    emit min(_min);
  }

  void setMax(double newMax) {
    _max = newMax;
    emit max(_max);
  }

public slots:

  void in(const TaggedMat& taggedMat);

signals:

  void seq(int seq);
  void cols(int cols);
  void rows(int rows);
  void channels(int channels);
  void type(int type);
  void depth(int depth);
  void min(double min);
  void max(double max);

protected:

  int _seq;
  int _cols;
  int _rows;
  int _channels;
  MatType _type;
  MatDepth _depth;
  double _min;
  double _max;

};

#endif // TaggedMatInfo_H
