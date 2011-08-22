/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleImageToImageMetric.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleImageToImageMetric_h
#define __itkSimpleImageToImageMetric_h

#include "itkImageToImageMetric.h"
#include "itkStrategyFactory.h"
#include "itkExceptionObject.h"

#include "itkCompareHistogramImageToImageMetric.h"
#include "itkCorrelationCoefficientHistogramImageToImageMetric.h"
#include "itkGradientDifferenceImageToImageMetric.h"
#include "itkKappaStatisticImageToImageMetric.h"
#include "itkKullbackLeiblerCompareHistogramImageToImageMetric.h"
#include "itkMatchCardinalityImageToImageMetric.h"
#include "itkMattesMutualInformationImageToImageMetric.h"
#include "itkMeanReciprocalSquareDifferenceImageToImageMetric.h"
#include "itkMeanSquaresHistogramImageToImageMetric.h"
#include "itkMeanSquaresImageToImageMetric.h"
#include "itkMutualInformationHistogramImageToImageMetric.h"
#include "itkMutualInformationImageToImageMetric.h"
#include "itkNormalizedCorrelationImageToImageMetric.h"
#include "itkNormalizedMutualInformationHistogramImageToImageMetric.h"

namespace itk
{

/** List of strategies for metrics. */
enum ImageToImageMetricStrategy {
  ImageToImageMetricStrategyCompareHistogram,
  ImageToImageMetricStrategyCorrelationCoefficientHistogram,
  ImageToImageMetricStrategyGradientDifference,
  ImageToImageMetricStrategyKappaStatistic,
  ImageToImageMetricStrategyKullbackLeiblerCompareHistogram,
  ImageToImageMetricStrategyMatchCardinality,
  ImageToImageMetricStrategyMattesMutualInformation,
  ImageToImageMetricStrategyMeanReciprocalSquareDifference,
  ImageToImageMetricStrategyMeanSquaresHistogram,
  ImageToImageMetricStrategyMeanSquares,
  ImageToImageMetricStrategyMutualInformationHistogram,
  ImageToImageMetricStrategyMutualInformation,
  ImageToImageMetricStrategyNormalizedCorrelation,
  ImageToImageMetricStrategyNormalizedMutualInformationHistogram
};

/** \class SimpleImageToImageMetric
 * \brief Implements various image to image metrics.
 *
 * This object is templated over the following types:
 *   TFixedImage: Required fixed image type
 *   TMovingImage: Optional moving image type
 *
 * This object expects the following inputs:
 *   FixedImage: The fixed image
 *   MovingImage: The moving image
 *   Transform: The transform
 *   TransformParameters: The transform parameters used to map the moving
 *     image onto the fixed image
 *   Interpolator: The interpolator
 *   FixedImageRegion: The region of the fixed image for computation
 *   FixedImageMask: Optional mask for the fixed image
 *   MovingImageMask: Optional mask for the moving image
 *
 * This object produces the following outputs:
 *   Value: The resultant metric value
 *   Derivative: The resultant metric derivative
 *
 * This object currently supports the following strategies:
 *   CompareHistogram: Computes the similarity between histograms.
 *   CorrelationCoefficientHistogram: Computes the correlation coefficient
 *     similarity measure using a histogram.
 *   GradientDifference: Computes the similarity using the sum of squared
 *     differences between pixels in the derivatives of the fixed and moving
 *     images
 *   KappaStatistic: Computes the similarity betwen two binary images using the
 *     formular: 2*|A&B|/(|A|+|B|), where A is the moving foreground region,
 *     B is the fixed foreground region, & is intersection, and |.| indicates
 *     the area of the enclosed set. See [1].
 *   KullbackLeiblerCompareHistogram: Computes the Kubler Lieblach(KL)
 *     similarity by comparing histograms. See [2].
 *   MatchCardinality: Computes the similarity by measuring the number of pixel
 *     matches (pixels with exactly the same value) or pixel mismatches (pixels
 *     with different values). The returned metric value is the number of pixel
 *     matches (or mismatches) normalized by the number of pixels considered.
 *     This metric is designed for using with binary or label images.
 *   MattesMutualInformation: Computes the similarity using mutual information
 *     (MI) where the probability density distribution are estimated using
 *     Parzen histograms. See [3], [4], and [5].
 *   MeanReciprocalSquareDifference: Computes the similarity using the sum of
 *     squared reciprocal (\f$ \frac{1}{1+x} \f$) difference.
 *   MeanSquaresHistogram: Computes the similarity using the mean of squared
 *     differences using a histogram.
 *   MeanSquares: Computes the similarity using the mean of squared
 *     differences.
 *   MutualInformationHistogram: Computes the similiarity as the mutual
 *     information between the two images using the histograms of the
 *     intensities in the images.
 *   MutualInformation: Computes the similarity using mutual information
 *     (MI) where the probability density distribution are estimated using
 *     Parzen histograms. See [6].
 *   NormalizedCorrelation: Computes the similarity using the normalized
 *     cross correlation between the two images.
 *   NormalizedMutualInformationHistogram: Computes normalized mutual
 *     information between the two images using the histograms of the
 *     intensities in the images. See [7].
 *
 * This object has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (RealType) ForegroundValue KappaStatistic
 *     Specifies pixel value considered as the foreground.
 *   (bool) Complement KappaStatistic
 *     Specifies whether the binary image is complemented/inverted.
 *   (bool) MeasureMatches MatchCardinality
 *     Specifies whether the metric measures pixel matches or pixel
 *     mismatches. The default is true (pixel matches).
 *   (bool) NumberOfThreads MatchCardinality
 *     Specifies the number of threads used by the metric.
 *   (unsigned long) NumberOfSpatialSamples MattesMutualInformation
 *     Specifies the number of spatial samples used to compute the MI.
 *   (unsigned long) NumberOfHistogramBins MattesMutualInformation
 *     Specifies the number of histogram bins used to compute the MI.
 *   (bool) UseAllPixels MattesMutualInformation
 *     Specifies whether the metric should use all pixels or a random sample.
 *   (bool) UseExplicitPDFDerivatives MattesMutualInformation
 *     Specifies whether the PDF deriviates are explicitly derived. See
 *     itkMattesMutualInformationImageToImageMetric.h for complete details.
 *   (bool) UseCachingOfBSplineWeights MattesMutualInformation
 *     When using BSplineDeformableTransform, specifies whether the BSpline
 *     weights are cached or not.
 *   (bool) SubtractMean NormalizedCorrelation
 *     Specifies whether the mean is subtracted from the sample values in the
 *     cross-correlation formula.
 *
 * References:
 * [1] Zijdenbos, Dawant, Margolin, and Palmer."Morphometric Analysis of White
 *     Matter Lesions in MR Images: Method and Validation"
 * [2] Chung, Wells, Norbash, and Grimson, "Multi-modal Image Registration by
 *     Minimising Kullback-Leibler Distance", In Medical Image Computing and
 *     Computer-Assisted Intervention (MICCAI), 2002, LNCS 2489, pp. 525--532.
 * [3] Mattes, Haynor, Vesselle, Lewellen and Eubank. "Nonrigid multimodality
 *     image registration", Medical Imaging 2001: Image Processing, 2001,
 *     pp. 1609--1620.
 * [4] Mattes, Haynor, Vesselle, Lewellen and Eubank. "PET-CT Image
 *     Registration in the Chest Using Free-form Deformations",
 *     IEEE Transactions in Medical Imaging. Vol.22, No.1, 2003, pp. 120--128.
 * [5] Thevenaz and Unser. "Optimization of Mutual Information for
 *     MultiResolution Image Registration", IEEE Transactions in Image
 *     Processing, Vol.9, No.12, 2000.
 * [6] Viola and Wells. "Alignment by Maximization of Mutual Information",
 *     International Journal of Computer Vision, Vol.24, No.2, 1997,
 *     pp. 137--154.
 * [7] Hajnal, Hill and Hawkes. "Medical Image Registration", Eq. 30, Chapter 3
 *     http://www-ipg.umds.ac.uk/d.hill/hhh/
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TFixedImage, class TMovingImage=TFixedImage>
class ITK_EXPORT SimpleImageToImageMetric :
  public ImageToImageMetric<TFixedImage,TMovingImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleImageToImageMetric Self;
  typedef ImageToImageMetric<TFixedImage, TMovingImage> Superclass;
  typedef SmartPointer<Self> Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleImageToImageMetric, ImageToImageMetric);

  /** Some typedefs. */
  typedef typename Superclass::MovingImageType MovingImageType;
  typedef typename Superclass::MovingImagePixelType MovingImagePixelType;
  typedef typename Superclass::MovingImageConstPointer MovingImageConstPointer;
  typedef typename Superclass::FixedImageType FixedImageType;
  typedef typename Superclass::FixedImageConstPointer FixedImageConstPointer;
  typedef typename Superclass::FixedImageRegionType FixedImageRegionType;
  typedef typename Superclass::TransformType TransformType;
  typedef typename Superclass::TransformPointer TransformPointer;
  typedef typename Superclass::InputPointType InputPointType;
  typedef typename Superclass::OutputPointType OutputPointType;
  typedef typename Superclass::RealType RealType;
  typedef typename Superclass::TransformParametersType TransformParametersType;
  typedef typename Superclass::TransformJacobianType TransformJacobianType;
  typedef typename Superclass::InterpolatorType InterpolatorType;
  typedef typename Superclass::InterpolatorPointer InterpolatorPointer;
  typedef typename Superclass::FixedImageMaskType FixedImageMaskType;
  typedef typename Superclass::FixedImageMaskPointer FixedImageMaskPointer;
  typedef typename Superclass::FixedImageMaskConstPointer FixedImageMaskConstPointer;
  typedef typename Superclass::MovingImageMaskType MovingImageMaskType;
  typedef typename Superclass::MovingImageMaskPointer MovingImageMaskPointer;
  typedef typename Superclass::MovingImageMaskConstPointer MovingImageMaskConstPointer;
  typedef typename Superclass::MeasureType MeasureType;
  typedef typename Superclass::DerivativeType DerivativeType;
  typedef typename Superclass::ParametersType ParametersType;
  itkStaticConstMacro(MovingImageDimension,
                      unsigned int,
                      TMovingImage::ImageDimension);
  itkStaticConstMacro(FixedImageDimension,
                      unsigned int,
                      TFixedImage::ImageDimension);

  /** Strategy factory typedefs */
  typedef ImageToImageMetricStrategy StrategyKeyType;
  typedef Superclass StrategyType;
  typedef StrategyFactory<StrategyKeyType,StrategyType> StrategyFactoryType;

  /** Get/set strategy key */
  StrategyKeyType GetStrategy()
    {
    return m_StrategyFactory.GetKey();
    }
  void SetStrategy(StrategyKeyType key)
    {
    m_StrategyFactory.SetKey(key);
    this->Modified();
    }

  /** Get/set the fixed image. */
  virtual void SetFixedImage(const FixedImageType* arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetFixedImage(arg);
  }
  virtual const FixedImageType* GetFixedImage() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetFixedImage();
  }

  /** Get/set the moving image. */
  virtual void SetMovingImage(const MovingImageType* arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetMovingImage(arg);
  }
  virtual const MovingImageType* GetMovingImage() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetMovingImage();
  }

  /** Get/set the transform. */
  virtual void SetTransform(TransformType* arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetTransform(arg);
  }
  virtual const TransformType* GetTransform() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetTransform();
  }

  /** Set the transform parameters. */
  void SetTransformParameters(const ParametersType & arg) const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetTransformParameters(arg);
  }

  /** Get number of parameters required by the Transform. */
  unsigned int GetNumberOfParameters(void) const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetNumberOfParameters();
  }
  /** Get/set the interpolator. */
  virtual void SetInterpolator(InterpolatorType* arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetInterpolator(arg);
  }
  virtual const InterpolatorType* GetInterpolator() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetInterpolator();
  }

  /** Get/set the fixed image region. */
  virtual void SetFixedImageRegion(FixedImageRegionType arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetFixedImageRegion(arg);
  }
  virtual const FixedImageRegionType& GetFixedImageRegion() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetFixedImageRegion();
  }

  /** Get/set the fixed image mask. */
  virtual void SetFixedImageMask(FixedImageMaskType* arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetFixedImageMask(arg);
  }
  virtual const FixedImageMaskType* GetFixedImageMask() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetFixedImageMask();
  }

  /** Get/set the moving image mask. */
  virtual void SetMovingImageMask(MovingImageMaskType* arg)
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->SetMovingImageMask(arg);
  }
  virtual const MovingImageMaskType* GetMovingImageMask() const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetMovingImageMask();
  }

  /** Initialize the Metric. */
  virtual void Initialize(void) throw ( ExceptionObject )
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->Initialize();
  }

  /**  Get the value of the metric. */
  MeasureType GetValue(const TransformParametersType & params) const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    return metric->GetValue(params);
  }

  /** Get the derivative of the metric. */
  void GetDerivative(const TransformParametersType& params,
                     DerivativeType& derivative ) const
  {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* metric = ptrStrategyFactory->GetStrategy();
    metric->GetDerivative(params, derivative);
  }

  /** KappaStatistic parameters. */
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, KappaStatisticImageToImageMetricType,
    KappaStatistic, ForegroundValue, RealType
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, KappaStatisticImageToImageMetricType,
    KappaStatistic, ForegroundValue, RealType
  );
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, KappaStatisticImageToImageMetricType,
    KappaStatistic, Complement, bool
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, KappaStatisticImageToImageMetricType,
    KappaStatistic, Complement, bool
  );

  /** MatchCardinality parameters. */
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MatchCardinalityImageToImageMetricType,
    MatchCardinality, MeasureMatches, bool
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MatchCardinalityImageToImageMetricType,
    MatchCardinality, MeasureMatches, bool
  );
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MatchCardinalityImageToImageMetricType,
    MatchCardinality, NumberOfThreads, int
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MatchCardinalityImageToImageMetricType,
    MatchCardinality, NumberOfThreads, int
  );

  /** MattesMutualInformation parameters. */
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, NumberOfSpatialSamples, unsigned long
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, NumberOfSpatialSamples, unsigned long
  );
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, NumberOfHistogramBins, unsigned long
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, NumberOfHistogramBins, unsigned long
  );
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, UseAllPixels, bool
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, UseAllPixels, bool
  );
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, UseExplicitPDFDerivatives, bool
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, UseExplicitPDFDerivatives, bool
  );
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, UseCachingOfBSplineWeights, bool
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, MattesMutualInformationImageToImageMetricType,
    MattesMutualInformation, UseCachingOfBSplineWeights, bool
  );

  /** NormalizedCorrelation parameters. */
  itkSetConcreteStrategyMacro(
    ImageToImageMetric, NormalizedCorrelationImageToImageMetricType,
    NormalizedCorrelation, SubtractMean, bool
  );
  itkGetConcreteStrategyMacro(
    ImageToImageMetric, NormalizedCorrelationImageToImageMetricType,
    NormalizedCorrelation, SubtractMean, bool
  );

protected:
  SimpleImageToImageMetric() : m_StrategyFactory(StrategyFactoryType(this))
    {
    m_StrategyFactory.template AddStrategy<CompareHistogramImageToImageMetricType>
      ( ImageToImageMetricStrategyCompareHistogram );
    m_StrategyFactory.template AddStrategy<CorrelationCoefficientHistogramImageToImageMetricType>
      ( ImageToImageMetricStrategyCorrelationCoefficientHistogram );
    m_StrategyFactory.template AddStrategy<GradientDifferenceImageToImageMetricType>
      ( ImageToImageMetricStrategyGradientDifference );
    m_StrategyFactory.template AddStrategy<KappaStatisticImageToImageMetricType>
      ( ImageToImageMetricStrategyKappaStatistic );
    m_StrategyFactory.template AddStrategy<KullbackLeiblerCompareHistogramImageToImageMetricType>
      ( ImageToImageMetricStrategyKullbackLeiblerCompareHistogram );
    m_StrategyFactory.template AddStrategy<MatchCardinalityImageToImageMetricType>
      ( ImageToImageMetricStrategyMatchCardinality );
    m_StrategyFactory.template AddStrategy<MattesMutualInformationImageToImageMetricType>
      ( ImageToImageMetricStrategyMattesMutualInformation );
    m_StrategyFactory.template AddStrategy<MeanReciprocalSquareDifferenceImageToImageMetricType>
      ( ImageToImageMetricStrategyMeanReciprocalSquareDifference );
    m_StrategyFactory.template AddStrategy<MeanSquaresHistogramImageToImageMetricType>
      ( ImageToImageMetricStrategyMeanSquaresHistogram );
    m_StrategyFactory.template AddStrategy<MeanSquaresImageToImageMetricType>
      ( ImageToImageMetricStrategyMeanSquares );
    m_StrategyFactory.template AddStrategy<MutualInformationHistogramImageToImageMetricType>
      ( ImageToImageMetricStrategyMutualInformationHistogram );
    m_StrategyFactory.template AddStrategy<NormalizedCorrelationImageToImageMetricType>
      ( ImageToImageMetricStrategyNormalizedCorrelation );
    m_StrategyFactory.template AddStrategy<NormalizedMutualInformationHistogramImageToImageMetricType>
      ( ImageToImageMetricStrategyNormalizedMutualInformationHistogram );
    }
  virtual ~SimpleImageToImageMetric() {};

private:
  SimpleImageToImageMetric(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef CompareHistogramImageToImageMetric<FixedImageType,MovingImageType>
    CompareHistogramImageToImageMetricType;
  typedef CorrelationCoefficientHistogramImageToImageMetric<FixedImageType,MovingImageType>
    CorrelationCoefficientHistogramImageToImageMetricType;
  typedef GradientDifferenceImageToImageMetric<FixedImageType,MovingImageType>
    GradientDifferenceImageToImageMetricType;
  typedef KappaStatisticImageToImageMetric<FixedImageType,MovingImageType>
    KappaStatisticImageToImageMetricType;
  typedef KullbackLeiblerCompareHistogramImageToImageMetric<FixedImageType,MovingImageType>
    KullbackLeiblerCompareHistogramImageToImageMetricType;
  typedef MatchCardinalityImageToImageMetric<FixedImageType,MovingImageType>
    MatchCardinalityImageToImageMetricType;
  typedef MattesMutualInformationImageToImageMetric<FixedImageType,MovingImageType>
    MattesMutualInformationImageToImageMetricType;
  typedef MeanReciprocalSquareDifferenceImageToImageMetric<FixedImageType,MovingImageType>
    MeanReciprocalSquareDifferenceImageToImageMetricType;
  typedef MeanSquaresHistogramImageToImageMetric<FixedImageType,MovingImageType>
    MeanSquaresHistogramImageToImageMetricType;
  typedef MeanSquaresImageToImageMetric<FixedImageType,MovingImageType>
    MeanSquaresImageToImageMetricType;
  typedef MutualInformationHistogramImageToImageMetric<FixedImageType,MovingImageType>
    MutualInformationHistogramImageToImageMetricType;
  typedef MutualInformationImageToImageMetric<FixedImageType,MovingImageType>
    MutualInformationImageToImageMetricType;
  typedef NormalizedCorrelationImageToImageMetric<FixedImageType,MovingImageType>
    NormalizedCorrelationImageToImageMetricType;
  typedef NormalizedMutualInformationHistogramImageToImageMetric<FixedImageType,MovingImageType>
    NormalizedMutualInformationHistogramImageToImageMetricType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
