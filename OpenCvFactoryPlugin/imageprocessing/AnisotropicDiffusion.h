#ifndef ANISOTROPICDIFFUSION_H
#define ANISOTROPICDIFFUSION_H

#include <QObject>
#include <opencv2/imgproc.hpp>

class AnisotropicDiffusion : public QObject
{
  Q_OBJECT

  Q_CLASSINFO("class-alias","Anisotropic Diffusion")
  Q_CLASSINFO("directory","OpenCV/Image Processing")

  Q_PROPERTY(double alpa READ alpha WRITE alpha)
  Q_PROPERTY(double sensitivity READ sensitivity WRITE sensitivity)
  Q_PROPERTY(int iterations READ iterations WRITE iterations)

public:

  Q_INVOKABLE explicit AnisotropicDiffusion(QObject* parent = nullptr);

  virtual ~AnisotropicDiffusion() {}

  double alpha() const { return _alpha; }
  double sensitivity() const { return _sensitivity; }
  int iterations() const { return _iterations; }

public slots:

  void in(const cv::Mat& mat);

  void alpha(double alpha);
  void sensitivity(double sensitivity);
  void iterations(int iterations);

signals:

  void out(const cv::Mat& mat);

protected:

  void update();

private:

  cv::Mat _input;
  double _alpha;
  double _sensitivity;
  int _iterations;

};

#endif // ANISOTROPICDIFFUSION_H
