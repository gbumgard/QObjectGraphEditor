#ifndef THRESHOLD_H
#define THRESHOLD_H

#include <QObject>
#include <QLoggingCategory>
#include <QVariant>
#include "AbstractOpenCvObject.h"
#include <opencv2/imgproc.hpp>

class ThresholdFilter : public AbstractOpenCvObject
{
  Q_OBJECT

  Q_PROPERTY(FilterType filterType READ filterType WRITE setFilterType NOTIFY filterTypeChanged)
  Q_PROPERTY(double threshold READ threshold WRITE threshold NOTIFY thresholdChanged)
  Q_PROPERTY(double maximum READ maximum WRITE maximum NOTIFY maximumChanged)

  Q_PROPERTY(bool enableDebugMsgs READ enableDebugMsgs WRITE setEnableDebugMsgs NOTIFY enableDebugMsgsChanged)
  Q_PROPERTY(bool enableInfoMsgs READ enableInfoMsgs WRITE setEnableInfoMsgs NOTIFY enableInfoMsgsChanged)
  Q_PROPERTY(bool enableWarningMsgs READ enableWarningMsgs WRITE setEnableWarningMsgs NOTIFY enableWarningMsgsChanged)
  Q_PROPERTY(bool enableCriticalMsgs READ enableCriticalMsgs WRITE setEnableCriticalMsgs NOTIFY enableCriticalMsgsChanged)

  Q_CLASSINFO("class-alias","Threshold Filter")
  Q_CLASSINFO("directory","OpenCV/Filters")

public:

  enum FilterType {
    THRESH_BINARY = cv::THRESH_BINARY,
    THRESH_BINARY_INV = cv::THRESH_BINARY_INV,
    THRESH_TRUNC = cv::THRESH_TRUNC,
    THRESH_TO_ZERO = cv::THRESH_TOZERO,
    THRESH_TOZERO_INV = cv::THRESH_TOZERO_INV
  };

  Q_ENUM(FilterType)

  Q_INVOKABLE explicit ThresholdFilter(QObject* parent = nullptr);

  ~ThresholdFilter() {}

  void setFilterType(FilterType filterType);
  FilterType filterType() const { return _filterType; }

  double threshold() const { return _threshold; }
  double maximum() const { return _maximum; }

  bool enableDebugMsgs() const { return _logcat.isDebugEnabled(); }
  void setEnableDebugMsgs(bool enable) { _logcat.setEnabled(QtDebugMsg, enable); }

  bool enableInfoMsgs() const { return _logcat.isInfoEnabled(); }
  void setEnableInfoMsgs(bool enable) { _logcat.setEnabled(QtInfoMsg, enable); }

  bool enableWarningMsgs() const { return _logcat.isWarningEnabled(); }
  void setEnableWarningMsgs(bool enable) { _logcat.setEnabled(QtWarningMsg, enable); }

  bool enableCriticalMsgs() const { return _logcat.isCriticalEnabled(); }
  void setEnableCriticalMsgs(bool enable) {_logcat.setEnabled(QtCriticalMsg, enable); }

public slots:

  QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);

  void threshold(double threshold);
  void maximum(double maximum);

signals:

  QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);

  void filterTypeChanged(const FilterType&);
  void thresholdChanged(double);
  void maximumChanged(double);

  void enableDebugMsgsChanged(bool);
  void enableInfoMsgsChanged(bool);
  void enableWarningMsgsChanged(bool);
  void enableCriticalMsgsChanged(bool);

protected:

  FilterType _filterType;
  double _threshold;
  double _maximum;

  QLoggingCategory _logcat;

};


#endif // THRESHOLD_H
