#ifndef MATFINGERPRINT_H
#define MATFINGERPRINT_H

#include <vector>
#include <opencv2/core.hpp>

class MatFingerprint
{
public:

  MatFingerprint(const cv::Mat& mat)
    : _fingerprint()
  {
    _fingerprint.push_back(mat.type());
    _fingerprint.push_back(mat.depth());
    _fingerprint.push_back(mat.channels());
    std::copy(mat.size.p,mat.size.p+mat.dims,std::back_inserter(_fingerprint));
    _hash = hash(_fingerprint);
  }

  MatFingerprint(const MatFingerprint& other)
    : _hash(other._hash), _fingerprint(other._fingerprint)
  {
  }

  MatFingerprint& operator=(const MatFingerprint& other) {
    _hash = other._hash;
    _fingerprint = other._fingerprint;
    return *this;
  }

  int type() const {
    return _fingerprint[TYPE_INDEX];
  }

  int depth() const {
    return _fingerprint[DEPTH_INDEX];
  }

  int channels() const {
    return _fingerprint[CHANNEL_INDEX];
  }

  int dims() const {
    return _fingerprint.size() - SIZES_INDEX;
  }

  std::vector<int> sizes() const {
    return std::vector<int>(_fingerprint.begin() + SIZES_INDEX,_fingerprint.end());
  }

  size_t hash() const {
    return _hash;
  }

  bool matches(const MatFingerprint& other) const {
    return _hash == other._hash && other._fingerprint == _fingerprint;
  }

  bool operator==(const MatFingerprint& other) const {
    return matches(other);
  }

  bool operator!=(const MatFingerprint& other) const {
    return !matches(other);
  }

  bool isSameHash(const MatFingerprint& other) {
    return hash() == other.hash();
  }

  bool isSameType(const MatFingerprint& other) const {
    return type() == other.type();
  }

  bool isSameDepth(const MatFingerprint& other) const {
    return depth() == other.depth();
  }

  bool isSameChannels(const MatFingerprint& other) const {
    return channels() == other.channels();
  }

  bool isSameDims(const MatFingerprint& other) const {
    return dims() == other.dims();
  }

  bool isSameSize(const MatFingerprint& other) const {
    return isSameDims(other) && std::equal(_fingerprint.begin()+SIZES_INDEX,_fingerprint.end(),other._fingerprint.begin()+SIZES_INDEX);
  }

  bool isSameShape(const MatFingerprint& other) const {
    return isSameChannels(other) && isSameSize(other);
  }

  template <class none = void>
  inline bool isOneOfType() const {
      return false;
  }

  template <int matType, int... matTypes>
  inline bool isOneOfType() const {
    return _fingerprint[TYPE_INDEX] == matType || isOneOfType<matTypes...>();
  }

  template <class none = void>
  inline bool isOneOfDepth() const {
      return false;
  }

  template <int depthType, int... depthTypes>
  inline bool isOneOfDepth() const {
    return _fingerprint[DEPTH_INDEX] == depthType || isOneOfDepth<depthTypes...>();
  }

  template <class none = void>
  inline bool isOneOfChannels() const {
      return false;
  }

  template <int channelCount, int... channelCounts>
  inline bool isOneOfChannels() const {
    return _fingerprint[CHANNEL_INDEX] == channelCount || isOneOfChannels<channelCounts...>();
  }

  bool canMask(const MatFingerprint& other) const {
    return channels() == 1 && isSameSize(other);
  }

  bool canMask(const cv::Mat& mat) const {
    return channels() == 1 && isSameSize(mat);
  }

  void write(std::ostream& out) {
    size_t iter = 0;
    out << "type=     " << _fingerprint[iter++] << std::endl;
    out << "depth=    " << _fingerprint[iter++] << std::endl;
    out << "channels= " << _fingerprint[iter++] << std::endl;
    out << "dims=     " << _fingerprint.size()-3 << std::endl;
    out << "sizes=    ";
    while (iter < _fingerprint.size()) {
      out << _fingerprint[iter++] << " ";
    }
    out << std::endl;
    out << "hash=     " << _hash << std::endl;
  }

protected:

  MatFingerprint();

  static size_t hash(const std::vector<int>& vec) {
    size_t result = 0;
    for (int i : vec) {
      result ^= i + 0x9e3779b9 + (result << 6) + (result >> 2);
    }
    return result;
  }

private:

  static const int TYPE_INDEX = 0;
  static const int DEPTH_INDEX = 1;
  static const int CHANNEL_INDEX = 2;
  static const int SIZES_INDEX = 3;

  size_t _hash;
  std::vector<int> _fingerprint;

};

#endif // MATFINGERPRINT_H
