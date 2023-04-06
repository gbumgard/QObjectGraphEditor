#ifndef TAGGEDVARIANT_H
#define TAGGEDVARIANT_H

#include "Event.h"

#include <QVariant>


/**
 * @brief The TaggedQVariant class
 */
class QVariantEvent : public Event<QVariant,QVariant> {
public:

  explicit QVariantEvent()
    :Event()
  {
  }

  /**
   * @brief TaggedQVariant
   * @param variant
   */
  QVariantEvent(const QVariant& variant)
    : Event(variant,0)
  {
  }

  /**
   * @brief TaggedQVariant
   * @param variant
   * @param tag
   */
  QVariantEvent(const QVariant& variant, int64_t timestamp)
    : Event(variant,timestamp)
  {
  }

  /**
   * @brief TaggedQVariant
   * @param that
   */
  QVariantEvent(const QVariantEvent& that)
    : Event(that)
  {
  }

  virtual ~QVariantEvent() {}

  /**
   * @brief operator =
   * @param that
   * @return
   */
  QVariantEvent& operator=(const QVariantEvent& that) {
    Event::operator=(that);
    return *this;
  }

  /**
   * @brief variant
   * @return
   */
  QVariant& variant() {
    return payload();
  }

  /**
   * @brief variant
   * @return
   */
  const QVariant& variant() const {
    return payload();
  }

  virtual const QVariant& hash() const override {
    return payload();
  }

protected:

private:

  /**
   * @brief _variant
   */
  QVariant _variant;

};

Q_DECLARE_METATYPE(QVariantEvent)


#endif // TAGGEDVARIANT_H
