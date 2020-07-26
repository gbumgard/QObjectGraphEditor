#ifndef Event_H
#define Event_H

#include <algorithm>
#include <stdint.h>
#include <chrono>

/**
 * @brief The Event class
 */
template <typename Payload, typename HashType>
class Event {

public:

  static int64_t now() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  static const long UNTIMED = 0;

  explicit Event()
    : _timestamp(-1)
  {
  }

  /**
   * @brief release
   */
  virtual void release() {
    _timestamp = UNTIMED;
  }

  /**
   * @brief payload
   * @return
   */
  Payload& payload() {
    return _payload;
  }

  /**
   * @brief payload
   * @return
   */
  const Payload& payload() const {
    return _payload;
  }

  /**
   * @brief timestamp
   * @return
   */
  int64_t timestamp() const {
    return _timestamp;
  }

  /**
   * @brief event
   * @return
   */
  virtual const HashType& hash() const = 0;

  /**
   * @brief isTimed
   * @return
   */
  bool isTimed() const {
    return _timestamp != UNTIMED;
  }

  template<typename Event1, typename Event2>
  static bool isSynchronized(const Event1& event1, const Event2& event2) {
    return event1.timestamp() == event2.timestamp() || event1.timestamp() == UNTIMED || event1.timestamp() == UNTIMED;
  }

  template<typename Event1, typename Event2, typename... Events>
  static bool isSynchronized(const Event1& event1, const Event2& event2, Events... events) {
    return isSynchronized(event1,event2) && isSynchronized(event2, events...);
  }

  template<typename Event1, typename Event2>
  static int64_t maxTimestamp(const Event1& event1, const Event2& event2) {
    return std::max(event1.timestamp(),event2.timestamp());
  }

  template<typename FirstEvent, typename... Events>
  static int64_t maxTimestamp(const FirstEvent& event, Events... events) {
    return std::max(event.timestamp(),maxTimestamp(events...));
  }

  bool isSynchronizedMatch(const Event& other) const {
    return hash() == other.hash() && isSynchronized(*this,other);
  }

protected:

  Event(const Payload& payload,
        int64_t timestamp = now())
    : _payload(payload)
    , _timestamp(timestamp)
  {
  }

  /**
   * @brief Event
   * @param that
   */
  Event(const Event& other)
    : _payload(other._payload)
    , _timestamp(other.timestamp())
  {
  }

  /**
   * @brief operator =
   * @param that
   * @return
   */
  Event& operator=(const Event& other) {
    _payload = other._payload;
    _timestamp = other.timestamp();
    return *this;
  }

private:

  Payload _payload;

  /**
   * @brief _timestamp
   * The time the event was created measured as elapsed microseconds since the Unix epoch
   * or zero if the event is an 'untimed' event where synchronization is not required.
   */
  int64_t _timestamp;

};


#endif // Event_H
