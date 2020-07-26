#ifndef ScalarEvent_H
#define ScalarEvent_H

#include "Event.h"

#include <opencv2/core.hpp>
#include <QVariant>
#include <QDebug>

/**
 * @brief The MatWrapper class
 */
class ScalarEvent : public Event<cv::Scalar,cv::Scalar> {

public:

  static int userType() {
    static int typeId = QVariant::fromValue(ScalarEvent()).userType();
    return typeId;
  }

  explicit ScalarEvent()
    : Event()
  {
  }

  /**
   * @brief MatWrapper
   * Constructs a MatWrapper for operations that allow the use of an unsynchronized Mat.
   * @param mat
   */
  ScalarEvent(const cv::Scalar& scalar)
    : Event(scalar)
  {
  }

  /**
   * @brief MatWrapper
   * Constructs a MatWrapper for operations that allow the use of an unsynchronized Mat.
   * @param mat
   */
  ScalarEvent(const cv::Scalar& scalar, int64_t timestamp)
    : Event(scalar,timestamp)
  {
  }

  /**
   * @brief MatWrapper
   * @param that
   */
  ScalarEvent(const ScalarEvent& other)
    : Event(other)
  {
  }

  /**
   * @brief operator =
   * @param that
   * @return
   */
  ScalarEvent& operator=(const ScalarEvent& other) {
    Event::operator=(other);
    return *this;
  }

  /**
   * @brief release
   */
  virtual void release() override {
    Event::release();
  }

  cv::Scalar& scalar() {
    return payload();
  }

  const cv::Scalar& scalar() const {
    return payload();
  }

  /**
   * @brief operator bool
   */
  operator bool() const {
    return true;
  }

  /**
   * @brief hash
   * @return
   */
  virtual const cv::Scalar& hash() const override {
    return scalar();
  }

};

Q_DECLARE_METATYPE(ScalarEvent)

inline QDebug operator<<(QDebug dbg, const cv::Scalar& scalar) {
  dbg.nospace() << "cv::Scalar(" << scalar[0] << "," << scalar[1] << "," << scalar[2] << "," << scalar[3] << ")";
  return dbg.maybeSpace();
}

inline QDebug operator<<(QDebug dbg, const ScalarEvent& scalarEvent) {
  dbg.nospace() << "ScalarEvent(scalar=" << scalarEvent.scalar() << ",timestamp=" << scalarEvent.timestamp() << ")";
  return dbg.maybeSpace();
}

#endif // ScalarEvent_H
