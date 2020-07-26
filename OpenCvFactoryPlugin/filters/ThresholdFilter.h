#ifndef THRESHOLD_H
#define THRESHOLD_H

#include <OpenCvFactoryPlugin.h>
#include <QObject>
#include <QLoggingCategory>
#include <opencv2/imgproc.hpp>

class ThresholdFilter : public QObject
{
  Q_OBJECT

  Q_PROPERTY(FilterType filterType READ filterType WRITE setFilterType)
  Q_PROPERTY(double threshold READ threshold WRITE threshold)
  Q_PROPERTY(double maximum READ maximum WRITE maximum)

  Q_PROPERTY(bool enableDebugMsgs READ enableDebugMsgs WRITE setEnableDebugMsgs)
  Q_PROPERTY(bool enableInfoMsgs READ enableInfoMsgs WRITE setEnableInfoMsgs)
  Q_PROPERTY(bool enableWarningMsgs READ enableWarningMsgs WRITE setEnableWarningMsgs)
  Q_PROPERTY(bool enableCriticalMsgs READ enableCriticalMsgs WRITE setEnableCriticalMsgs)

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

  void in(const MatEvent& event);

  void threshold(double threshold);
  void maximum(double maximum);

signals:

  void out(const MatEvent& mat);

protected:

  FilterType _filterType;
  double _threshold;
  double _maximum;

  QLoggingCategory _logcat;

};


#endif // THRESHOLD_H
