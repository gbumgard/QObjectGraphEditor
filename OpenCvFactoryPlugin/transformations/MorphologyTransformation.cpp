#include "MorphologyTransformation.h"
#include "ObjectModel.h"

REGISTER_CLASS(MorphologyTransformation)

MorphologyTransformation::MorphologyTransformation(QObject* parent)
  : AbstractOpenCvObject(parent)
  , _operation(MorphologyTransformation::OPEN)
  , _shape(MorphologyTransformation::ELLIPSE)
  , _kernelSize(KERNEL_3x3)
  , _iterations(1)
{
}

void MorphologyTransformation::operation(Operation operation) {
  _operation = operation;
}

void MorphologyTransformation::shape(Shape shape) {
  _shape = shape;
}

void MorphologyTransformation::kernelSize(KernelSize kernelSize) {
  _kernelSize = kernelSize;
}

void MorphologyTransformation::iterations(int iterations) {
  _iterations = iterations >= 0 ? iterations : 0;
}

static const int rect3x3[3][3] = {
    { 1, 1, 1},
    { 1, 1, 1},
    { 1, 1, 1},
};

static const int ellipse3x3[3][3] = {
    { 0, 1, 0},
    { 1, 1, 1},
    { 0, 1, 0},
};

static const int cross3x3[3][3] = {
    { 0, 1, 0},
    { 1, 1, 1},
    { 0, 1, 0},
};

static const int rect5x5[5][5] = {
    { 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1}
};

static const int ellipse5x5[5][5] = {
    { 0, 0, 1, 0, 0},
    { 0, 1, 1, 1, 0},
    { 1, 1, 1, 1, 1},
    { 0, 1, 1, 1, 0},
    { 0, 0, 1, 0, 0}
};

static const int cross5x5[5][5] = {
    { 0, 0, 1, 0, 0},
    { 0, 0, 1, 0, 0},
    { 1, 1, 1, 1, 1},
    { 0, 0, 1, 0, 0},
    { 0, 0, 1, 0, 0}
};

static const int rect7x7[7][7] = {
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
};

static const int ellipse7x7[7][7] = {
    { 0, 0, 1, 1, 1, 0, 0},
    { 0, 1, 1, 1, 1, 1, 0},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 1, 1, 1, 1, 1, 1, 1},
    { 0, 1, 1, 1, 1, 1, 0},
    { 0, 0, 1, 1, 1, 0, 0},
 };

static const int cross7x7[7][7] = {
    { 0, 0, 0, 1, 0, 0, 0},
    { 0, 0, 0, 1, 0, 0, 0},
    { 0, 0, 0, 1, 0, 0, 0},
    { 1, 1, 1, 1, 1, 1, 1},
    { 0, 0, 0, 1, 0, 0, 0},
    { 0, 0, 0, 1, 0, 0, 0},
    { 0, 0, 0, 1, 0, 0, 0},
};

/*
cv::Mat MorphologyTransformation::getStructuringElement(Shape shape, KernelSize kernelSize) {
  cv::Mat element;
  switch (shape) {
  case RECT:
      switch (kernelSize) {
      case KERNEL_3x3:
        element = cv::Mat(3,3,CV_8UC1,(void*)rect3x3);
        break;
      case KERNEL_5x5:
        element = cv::Mat(5,5,CV_8UC1,(void*)rect5x5);
        break;
      case KERNEL_7x7:
        element = cv::Mat(7,7,CV_8UC1,(void*)rect7x7);
        break;
      }
      break;
  case ELLIPSE:
      switch (kernelSize) {
      case KERNEL_3x3:
        element = cv::Mat(3,3,CV_8UC1,(void*)ellipse3x3);
        break;
      case KERNEL_5x5:
        element = cv::Mat(5,5,CV_8UC1,(void*)ellipse5x5);
        break;
      case KERNEL_7x7:
        element = cv::Mat(7,7,CV_8UC1,(void*)ellipse7x7);
        break;
      }
      break;
  case CROSS:
      switch (kernelSize) {
      case KERNEL_3x3:
        element = cv::Mat(3,3,CV_8UC1,(void*)cross3x3);
        break;
      case KERNEL_5x5:
        element = cv::Mat(5,5,CV_8UC1,(void*)cross5x5);
        break;
      case KERNEL_7x7:
        element = cv::Mat(7,7,CV_8UC1,(void*)cross7x7);
        break;
      }
      break;
  }
  return element;
}
*/

#include "opencv2/imgproc.hpp"

cv::Mat MorphologyTransformation::getStructuringElement(Shape shape, KernelSize kernelSize, cv::Point anchor)
{
  int i, j;
  int r = 0, c = 0;
  double inv_r2 = 0;

  cv::Size ksize(kernelSize,kernelSize);

  CV_Assert( shape == (Shape)cv::MORPH_RECT || shape == (Shape)cv::MORPH_CROSS || shape == (Shape)cv::MORPH_ELLIPSE );

  if (anchor.x < 0) anchor.x = kernelSize >> 1;
  if (anchor.y < 0) anchor.y = kernelSize >> 1;

  if( ksize == cv::Size(1,1) )
      shape = (Shape)cv::MORPH_RECT;

  if( shape == (Shape)cv::MORPH_ELLIPSE )
  {
      r = ksize.height/2;
      c = ksize.width/2;
      inv_r2 = r ? 1./((double)r*r) : 0;
  }

  cv::Mat elem(ksize, CV_8U);

  for( i = 0; i < ksize.height; i++ )
  {
      uchar* ptr = elem.ptr(i);
      int j1 = 0, j2 = 0;

      if( shape == (Shape)cv::MORPH_RECT || (shape == (Shape)cv::MORPH_CROSS && i == anchor.y) )
        j2 = ksize.width;
      else if( shape == (Shape)cv::MORPH_CROSS )
        j1 = anchor.x, j2 = j1 + 1;
      else
      {
        int dy = i - r;
        if( std::abs(dy) <= r )
        {
            int dx = cv::saturate_cast<int>(c*std::sqrt((r*r - dy*dy)*inv_r2));
            j1 = std::max( c - dx, 0 );
            j2 = std::min( c + dx + 1, ksize.width );
        }
      }

      for( j = 0; j < j1; j++ )
        ptr[j] = 0;
      for( ; j < j2; j++ )
        ptr[j] = 1;
      for( ; j < ksize.width; j++ )
        ptr[j] = 0;
  }

  return elem;
}

void MorphologyTransformation::in(const QVariant &variant) {
  if (variant.userType() == MatEvent::userType()) {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_OK,"OK");
      MatEvent matEvent = qvariant_cast<MatEvent>(variant);
      //int kernelSize = (int)_kernelSize;
      cv::Mat structuringElement = getStructuringElement(_shape, _kernelSize);
      //cv::getStructuringElement(_shape, cv::Size(kernelSize,kernelSize));
      cv::Mat output;
      if (_iterations > 0) {
          cv::morphologyEx(matEvent.mat(),output,_operation,structuringElement,cv::Point(-1,-1),_iterations);
      }
      else  {
          output = matEvent.mat();
      }
      emit out(QVariant::fromValue(MatEvent(output,matEvent.timestamp())));
  }
  else {
      ObjectModel::setObjectStatus(this,ObjectModel::STATUS_INVALID_SLOT_ARGUMENT_FORMAT,"Unsupported SRC");
  }
}
