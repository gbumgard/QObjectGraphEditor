#ifndef MatCompare_H
#define MatCompare_H

#include <QObject>
#include <QVariant>
#include "MatEvent.h"
#include "AbstractOpenCvObject.h"
#include <opencv2/core.hpp>

class MatCompare : public AbstractOpenCvObject
{

    Q_OBJECT

    Q_CLASSINFO("class-alias","Mat Compare")
    Q_CLASSINFO("directory","OpenCV/Image Processing")

    Q_PROPERTY(double weight READ weight WRITE setWeight NOTIFY weightChanged)
    Q_PROPERTY(double errorThreshold READ errorThreshold WRITE setErrorThreshold NOTIFY errorThresholdChanged)
    Q_PROPERTY(int keyFrameInterval READ keyFrameInterval WRITE setKeyFrameInterval NOTIFY keyFrameIntervalChanged)
    Q_PROPERTY(int blockSize READ blockSize WRITE setBlockSize NOTIFY blockSizeChanged)
    Q_PROPERTY(int enableMask READ enableMask WRITE setEnableMask NOTIFY enableMaskChanged)

public:

    Q_INVOKABLE explicit MatCompare(QObject* parent = nullptr);

    virtual ~MatCompare() {}

    static double sigma(cv::Mat & m, int i, int j, int block_size = 1);
    static double cov(cv::Mat & m0, cv::Mat & m1, int i, int j, int block_size = 1);
    static double mse(cv::Mat & m0, cv::Mat & m1, bool grayscale = false, bool rooted = false, bool enableMask = false);
    static double rmse(cv::Mat & m0, cv::Mat & m1, bool enableMask = false);
    static double psnr(cv::Mat & m0, cv::Mat & m1, int block_size = 1);
    static double ssim(cv::Mat & m0, cv::Mat & m1, int block_size = 1, bool enableMask = false);

    double weight() const { return _filterWeight; }
    void setWeight(double weight) { _filterWeight = weight >= 0 ? weight : 0; }

    double errorThreshold() const { return _errorThreshold; }
    void setErrorThreshold(double threshold) { _errorThreshold = threshold >= 0 ? threshold : 0; }

    int keyFrameInterval() const { return _keyFrameInterval; }
    void setKeyFrameInterval(int interval) { _keyFrameInterval = interval >= 0 ? interval : 0; }

    int blockSize() const { return _blockSize; }
    void setBlockSize(int size) { _blockSize = size >= 0 ? size : 0; }

    bool enableMask() const { return _enableMask; }
    void setEnableMask(bool enable) { _enableMask = enable; }


public slots:

    QVARIANT_PAYLOAD(MatEvent) void in(const QVariant& srcEvent);
    QVARIANT_PAYLOAD(MatEvent) void ref(const QVariant& srcEvent);

signals:

    QVARIANT_PAYLOAD(MatEvent) void out(const QVariant& dstEvent);
    QVARIANT_PAYLOAD(MatEvent) void absDiff(const QVariant& dstEvent);

    void sigma(double);
    void cov(double);
    void mse(double);
    void rmse(double);
    void psnr(double);
    void ssim(double);

    void diff(double);
    void weightChanged(double);
    void errorThresholdChanged(double);
    void keyFrameIntervalChanged(int);
    void blockSizeChanged(int);
    void enableMaskChanged(bool);

private:

    int _keyFrameInterval;
    int _blockSize;
    bool _enableMask;

    double _filterWeight;
    double _filteredError;
    double _errorThreshold;

    int _frameCount;

    cv::Mat _reference;
};

#endif // MatCompare_H
