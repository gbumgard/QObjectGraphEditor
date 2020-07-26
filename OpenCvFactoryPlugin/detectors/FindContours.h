#ifndef FINDCONTOURS_H
#define FINDCONTOURS_H

#include <QObject>
#include <QMetaType>
#include <QPoint>
#include "ThreadedObject.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

Q_DECLARE_METATYPE(std::vector<std::vector<cv::Point>>)

class FindContours : public ThreadedObject
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Find Contours")
  Q_CLASSINFO("directory","OpenCV/Detectors")

  Q_PROPERTY(Mode mode READ mode WRITE mode)
  Q_PROPERTY(Method method READ method WRITE method)
  Q_PROPERTY(int firstContour READ firstContour WRITE firstContour)
  Q_PROPERTY(int lastContour READ lastContour WRITE lastContour)
  Q_PROPERTY(int stepSize READ stepSize WRITE stepSize)
  Q_PROPERTY(QPoint offset READ offset WRITE offset)

public:

  enum Mode {
    /**
     * Retrieves only the extreme outer contours. It sets hierarchy[i][2]=hierarchy[i][3]=-1 for all the contours.
     */
    RETR_EXTERNAL = cv::RETR_EXTERNAL,

    /**
     * Retrieves all of the contours without establishing any hierarchical relationships.
     */
    RETR_LIST = cv::RETR_LIST,

    /**
     * Retrieves all of the contours and organizes them into a two-level hierarchy.
     * At the top level, there are external boundaries of the components.
     * At the second level, there are boundaries of the holes.
     * If there is another contour inside a hole of a connected component, it is still put at the top level.
     */
    RETR_CCOMP = cv::RETR_CCOMP,

    /**
     * Retrieves all of the contours and reconstructs a full hierarchy of nested contours.
     */
    RETR_TREE = cv::RETR_TREE
  };

  Q_ENUM(Mode)

  enum Method {
    /**
     * Stores absolutely all the contour points - any 2 subsequent points (x1,y1) and (x2,y2) of the
     * contour will be either horizontal, vertical or diagonal neighbors, that is, max(abs(x1-x2),abs(y2-y1))==1.
     */
    CHAIN_APPROX_NONE = cv::CHAIN_APPROX_NONE,

    /**
     * Compresses horizontal, vertical, and diagonal segments and leaves only their end points.
     * For example, an up-right rectangular contour is encoded with 4 points.
     */
    CHAIN_APPROX_SIMPLE = cv::CHAIN_APPROX_SIMPLE,

    /**
     * Applies one of the flavors of the Teh-Chin chain approximation algorithm. See [TehChin89] for details.
     */
    CHAIN_APPROX_TC89_L1 = cv::CHAIN_APPROX_TC89_L1,

    /**
     * Applies one of the flavors of the Teh-Chin chain approximation algorithm. See [TehChin89] for details.
     */
    CHAIN_APPROX_TC89_KCOS = cv::CHAIN_APPROX_TC89_KCOS
  };

  Q_ENUM(Method)

  Q_INVOKABLE explicit FindContours(QObject *parent = nullptr);

  ~FindContours();

  Mode mode() const { return _mode; }

  Method method() const { return _method; }

  QPoint offset() const { return _offset; }

  int firstContour() const { return _firstContour; }

  int lastContour() const { return _lastContour; }

  int stepSize() const { return _stepSize; }

signals:

  void contours(const std::vector<std::vector<cv::Point>>& contours);

  void source(const cv::Mat& mat);

public slots:

  void in(const cv::Mat& mat);

  void mode(Mode mode);

  void method(Method method);

  void offset(const QPoint& offset);

  void firstContour(int firstContour);

  void lastContour(int lastContour);

  void stepSize(int stepSize);

protected:

  void update() override;

private:

  cv::Mat _nextInput;

  Mode _mode;
  Method _method;
  QPoint _offset;
  int _firstContour;
  int _lastContour;
  int _stepSize;

};

#endif // FINDCONTOURS_H
