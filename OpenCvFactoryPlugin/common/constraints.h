#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include "MatEvent.h"

/**
 * @brief The IsNotEmpty class
 */
class IsNotEmpty {
public:

  inline bool operator()(const MatEvent& target) const {
    return !target.payload().empty();
  }

};


/**
 * @brief The IsNotStatic class
 */
class IsNotStatic {
public:
  inline bool operator()(const MatEvent& target) const {
    return target.timestamp() != -1;
  }
};


/**
 * @brief The HasType class
 */
class HasType {
public:

  HasType(int type) : _type(type) {}

  inline bool operator()(const MatEvent& arg) const {
    return arg.payload().type() == _type;
  }

protected:

  int _type;

};

/**
 *
 */
template <int...>
class IsOneOfType;

/**
 *
 */
template <>
class IsOneOfType<> {
public:

  inline bool operator()(const MatEvent&) const {
    return false;
  }

};

/**
 *
 */
template <int matType, int... matTypes>
class IsOneOfType<matType, matTypes...> : public IsOneOfType<matTypes...> {
public:

  inline bool operator()(const MatEvent& obj) const {
    return obj.payload().type() == matType || IsOneOfType<matTypes...>()(obj);
  }

};


/**
 * @brief The HasDepth class
 */
class HasDepth {
public:

  HasDepth(int depth) : _depth(depth) {}

  inline bool operator()(const MatEvent& arg) const {
    return arg.payload().depth() == _depth;
  }

protected:

  int _depth;

};

/**
 *
 */
template <int...>
class IsOneOfDepth;

/**
 *
 */
template <>
class IsOneOfDepth<> {
  public:
  inline bool operator()(const MatEvent&) const {
    return false;
  }
};

/**
 *
 */
template <int depthType, int... depthTypes>
class IsOneOfDepth<depthType, depthTypes...> : public IsOneOfDepth<depthTypes...> {
  public:
  inline bool operator()(const MatEvent& obj) const {
    return (obj.payload().depth() == depthType || IsOneOfDepth<depthTypes...>()(obj));
  }
};


/**
 * @brief The HasChannelCount class
 */
class HasChannelCount {
public:

  HasChannelCount(int channels) : _channels(channels) {}

  inline bool operator()(const MatEvent& arg) const {
    return arg.payload().channels() == _channels;
  }

protected:

  int _channels;

};

/**
 *
 */
template <int...>
class IsOneOfChannelCount;

/**
 *
 */
template <>
class IsOneOfChannelCount<> {
  public:
  inline bool operator()(const MatEvent&) const {
    return false;
  }
};

/**
 *
 */
template <int channel, int... channels>
class IsOneOfChannelCount<channel, channels...> : public IsOneOfChannelCount<channels...> {
  public:
  inline bool operator()(const MatEvent& obj) const {
    return (obj.payload().channels() == channel || IsOneOfDepth<channels...>()(obj));
  }
};


/**
 * @brief The IsSameType class
 */
class IsSameType {
  public:
  inline bool operator()(const MatEvent& arg1, const MatEvent& arg2) const {
    return arg1.payload().type() == arg2.payload().type();
  }
};


/**
 * @brief The IsSameDepth class
 */
class IsSameDepth {
  public:
  inline bool operator()(const MatEvent& arg1, const MatEvent& arg2) const {
    return arg1.payload().depth() == arg2.payload().depth();
  }
};


/**
 * @brief The IsSameSize class
 */
class IsSameSize {
  public:
  inline bool operator()(const MatEvent& arg1, const MatEvent& arg2) const {
    return arg1.payload().size() == arg2.payload().size();
  }
};

class IsArithScalarOpSizeMatch {
public:
  bool operator()(const MatEvent& src1, const MatEvent& src2) {
    return (src1.payload().size() == src2.payload().size() ||
            src1.payload().channels() == (long)src2.payload().total()*src2.payload().channels() ||
            src2.payload().channels() == (long)src1.payload().total()*src1.payload().channels());
  }
};

class IsArithScalarOpWithMaskSizeMatch {
public:
  bool operator()(const MatEvent& src1, const MatEvent& src2, const MatEvent& mask) {
    return src1.payload().size() == src2.payload().size() && (mask.payload().empty() || src1.payload().size() == mask.payload().size() || src1.payload().size() == mask.payload().size());
  }
};


template<typename Arg1, typename Arg2>
bool isEqual(const Arg1& arg1, const Arg2& arg2) {
  return arg1 == arg2;
}

template<typename Arg1, typename Arg2, typename... Args>
bool isEqual(const Arg1& arg1, const Arg2& arg2, Args... args) {
  return arg1 == arg2 && isEqual(arg2, args...);
}




typedef IsOneOfType<CV_8UC1> IsCV_8UC1;
typedef IsOneOfType<CV_8UC2> IsCV_8UC2;
typedef IsOneOfType<CV_8UC3> IsCV_8UC3;
typedef IsOneOfType<CV_8UC4> IsCV_8UC4;
typedef IsOneOfType<CV_8SC1> IsCV_8SC1;
typedef IsOneOfType<CV_8SC2> IsCV_8SC2;
typedef IsOneOfType<CV_8SC3> IsCV_8SC3;
typedef IsOneOfType<CV_8SC4> IsCV_8SC4;
typedef IsOneOfType<CV_16UC1> IsCV_16UC1;
typedef IsOneOfType<CV_16UC2> IsCV_16UC2;
typedef IsOneOfType<CV_16UC3> IsCV_16UC3;
typedef IsOneOfType<CV_16UC4> IsCV_16UC4;
typedef IsOneOfType<CV_16SC1> IsCV_16SC1;
typedef IsOneOfType<CV_16SC2> IsCV_16SC2;
typedef IsOneOfType<CV_16SC3> IsCV_16SC3;
typedef IsOneOfType<CV_16SC4> IsCV_16SC4;
typedef IsOneOfType<CV_32SC1> IsCV_32SC1;
typedef IsOneOfType<CV_32SC2> IsCV_32SC2;
typedef IsOneOfType<CV_32SC3> IsCV_32SC3;
typedef IsOneOfType<CV_32SC4> IsCV_32SC4;
typedef IsOneOfType<CV_32FC1> IsCV_32FC1;
typedef IsOneOfType<CV_32FC1> IsCV_32FC1;
typedef IsOneOfType<CV_32FC2> IsCV_32FC2;
typedef IsOneOfType<CV_32FC3> IsCV_32FC3;
typedef IsOneOfType<CV_64FC1> IsCV_64FC1;
typedef IsOneOfType<CV_64FC2> IsCV_64FC2;
typedef IsOneOfType<CV_64FC3> IsCV_64FC3;
typedef IsOneOfType<CV_64FC4> IsCV_64FC4;
typedef IsOneOfType<CV_16FC1> IsCV_16FC1;
typedef IsOneOfType<CV_16FC2> IsCV_16FC2;
typedef IsOneOfType<CV_16FC3> IsCV_16FC3;
typedef IsOneOfType<CV_16FC4> IsCV_16FC4;

typedef IsOneOfDepth<CV_8U> IsCV_8U;
typedef IsOneOfDepth<CV_8S> IsCV_8S;
typedef IsOneOfDepth<CV_16U> IsCV_16U;
typedef IsOneOfDepth<CV_16S> IsCV_16S;
typedef IsOneOfDepth<CV_32S> IsCV_32S;
typedef IsOneOfDepth<CV_32F> IsCV_32F;
typedef IsOneOfDepth<CV_64F> IsCV_64F;
typedef IsOneOfDepth<CV_64F> IsCV_16F;

#endif // CONSTRAINTS_H
