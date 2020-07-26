#ifndef MatEvent_H
#define MatEvent_H

#include "Event.h"

#include <opencv2/core.hpp>
#include <QVariant>
#include <QDebug>

/**
 * @brief The MatWrapper class
 */
class MatEvent : public Event<cv::Mat,size_t> {

public:

  static int userType() {
    static int typeId = QVariant::fromValue(MatEvent()).userType();
    return typeId;
  }

  explicit MatEvent()
    : Event()
  {
  }

  /**
   * @brief MatWrapper
   * Constructs a MatWrapper for operations that allow the use of an unsynchronized Mat.
   * @param mat
   */
  MatEvent(const cv::Mat& mat)
    : Event(mat)
    , _hash(computeHash(mat))
  {
  }

  /**
   * @brief MatWrapper
   * Constructs a MatWrapper for operations that allow the use of an unsynchronized Mat.
   * @param mat
   */
  MatEvent(const cv::Mat& mat, int64_t timestamp)
    : Event(mat,timestamp)
    , _hash(computeHash(mat))
  {
  }

  /**
   * @brief MatWrapper
   * @param that
   */
  MatEvent(const MatEvent& other)
    : Event(other)
    , _hash(other._hash)
  {
  }

  /**
   * @brief operator =
   * @param that
   * @return
   */
  MatEvent& operator=(const MatEvent& other) {
    Event::operator=(other);
    _hash = other._hash;
    return *this;
  }

  /**
   * @brief release
   */
  virtual void release() override {
    Event::release();
    payload().release();
  }

  cv::Mat& mat() {
    return payload();
  }

  const cv::Mat& mat() const {
    return payload();
  }

  /**
   * @brief operator bool
   */
  operator bool() const {
    return !payload().empty();
  }

  virtual const size_t& hash() const override {
    return _hash;
  }

  /**
   * @brief isSynchronizedMask
   * @param other
   * @return
   */
  bool isSynchronizedMask(const MatEvent& mask) const {
    return mask && MatEvent::isSynchronized(*this,mask) && mat().size() == mask.mat().size();
  }

protected:

  static size_t computeHash(const cv::Mat& mat) {
    std::vector<int> fingerprint;
    fingerprint.push_back(mat.type());
    fingerprint.push_back(mat.depth());
    fingerprint.push_back(mat.channels());
    std::copy(mat.size.p,mat.size.p+mat.dims,std::back_inserter(fingerprint));
    size_t result = 0;
    for (int i : fingerprint) {
      result ^= i + 0x9e3779b9 + (result << 6) + (result >> 2);
    }
    return result;
  }

  size_t _hash;
};

Q_DECLARE_METATYPE(MatEvent)

inline QDebug operator<<(QDebug dbg, const cv::Mat& mat) {
  dbg.nospace() << "cv::Mat(rows=" << mat.rows << ",cols=" << mat.cols << ",type=" << mat.type() << ",depth=" << mat.depth() << ",channels=" << mat.channels() << ")";
  return dbg.maybeSpace();
}

inline QDebug operator<<(QDebug dbg, const MatEvent matEvent) {
  dbg.nospace() << "MatEvent(mat=" << matEvent.mat() << ",hash=" << matEvent.hash() << ",timestamp=" << matEvent.timestamp() << ")";
  return dbg.maybeSpace();
}

#endif // MatEvent_H
