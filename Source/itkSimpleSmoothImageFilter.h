/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleSmoothImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleSmoothImageFilter_h
#define __itkSimpleSmoothImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkStrategyFactory.h"

#include "itkBilateralImageFilter.h"
#include "itkCurvatureFlowImageFilter.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkMedianImageFilter.h"

namespace itk
{

/** List of strategies for smoothing image filtering. */
enum SmoothStrategy {
  SmoothStrategyBilateral,
  SmoothStrategyCurvatureFlow,
  SmoothStrategyDiscreteGaussian,
  SmoothStrategyRecursiveGaussian,
  SmoothStrategyMedian
};

/** \class SimpleSmoothImageFilter
 * \brief Implements various smoothing image filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   ToutputImage: Optional output image type
 *
 * This filter expects the following inputs:
 *   Input1: The image to smooth or denoise
 *
 * This filter produces the following outputs:
 *   Output1: The resultant smoothed or denoised image
 *
 * This filter currently supports the following strategies:
 *   Bilateral: see [1]
 *   CurvatureFlow: see [2]
 *   DiscreteGaussian: see [3]
 *   RecursiveGaussian: see [4]
 *   Median: output pixel is the median of the surrounding neighborhood
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (ArrayType) DomainSigma Bilateral
 *     TODO:
 *   (double) RangeSigma Bilateral
 *     TODO:
 *   (unsigned int) FilterDimensionality Bilateral
 *     TODO:
 *   (unsigned long) NumberOfRangeGaussianSamples Bilateral
 *     TODO:
 *   (TimeStepType) TimeStep CurvatureFlow
 *     TODO:
 *   (unsigned int) NumberOfIterations CurvatureFlow
 *     TODO:
 *   (bool) UseImageSpacing CurvatureFlow
 *     TODO:
 *   (ArrayType) Variance DiscreteGaussian
 *     TODO:
 *   (ArrayType) MaximumError DiscreteGaussian
 *     TODO:
 *   (int) MaximumKernelWidth DiscreteGaussian
 *     TODO:
 *   (unsigned int) FilterDimensionality DiscreteGaussian
 *     TODO:
 *   (bool) UseImageSpacing DiscreteGaussian
 *     TODO:
 *   (ScalarRealType) Sigma RecursiveGaussian
 *     TODO:
 *   (bool) NormalizeAcrossScale RecursiveGaussian
 *     TODO:
 *   (InputSizeType) Radius Median
 *     TODO:
 *
 * References:
 * [1] Tomasi and Manduchi, "Bilateral Filtering for Gray and ColorImages."
 *     IEEE ICCV, 1998.
 * [2] Sethian, "Level Set Methods and Fast Marching Methods"
 *     Cambridge Press, Chapter 16, Second edition, 1999.
 * [3] Tony Lindeberg, "Discrete Scale-Space Theory and the Scale-Space
 *     Primal Sketch", Dissertation. Royal Institute of Technology,
 *     Stockholm, Sweden. May 1991.
 * [4] Deriche, "Recursively Implementing The Gaussian and Its Derivatives",
 *     INRIA, 1993, ftp://ftp.inria.fr/INRIA/tech-reports/RR/RR-1893.ps.gz
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 * */
template <class TInputImage,class TOutputImage=TInputImage>
class ITK_EXPORT SimpleSmoothImageFilter :
  public ImageToImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleSmoothImageFilter                       Self;
  typedef ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleSmoothImageFilter, ImageToImageFilter);

  /** Some typedefs. */
  typedef TInputImage                              InputImageType;
  typedef typename    InputImageType::ConstPointer InputImagePointer;
  typedef typename    InputImageType::RegionType   InputRegionType;
  typedef typename    InputImageType::PixelType    InputPixelType;
  typedef TOutputImage                             OutputImageType;
  typedef typename     OutputImageType::Pointer    OutputImagePointer;
  typedef typename     OutputImageType::RegionType OutputRegionType;
  typedef typename     OutputImageType::PixelType  OutputPixelType;

  /** Strategy factory typedefs */
  typedef SmoothStrategy StrategyKeyType;
  typedef Superclass StrategyType;
  typedef StrategyFactory<StrategyKeyType,StrategyType> StrategyFactoryType;

  /** Strategy typedefs */
  typedef BilateralImageFilter<InputImageType,OutputImageType>
    BilateralType;
  typedef CurvatureFlowImageFilter<InputImageType,OutputImageType>
    CurvatureFlowType;
  typedef DiscreteGaussianImageFilter<InputImageType,OutputImageType>
    DiscreteGaussianType;
  typedef SmoothingRecursiveGaussianImageFilter<InputImageType,OutputImageType>
    RecursiveGaussianType;
  typedef MedianImageFilter<InputImageType,OutputImageType>
    MedianType;

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

  /** Get/set Bilateral filter parameters */
  itkSetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, DomainSigma, typename BilateralType::ArrayType
  );
  itkSetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, DomainSigma, double
  );
  itkGetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, DomainSigma, typename BilateralType::ArrayType
  );
  itkSetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, RangeSigma, double
  );
  itkGetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, RangeSigma, double
  );
  itkSetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, FilterDimensionality, unsigned int
  );
  itkGetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, FilterDimensionality, unsigned int
  );
  itkSetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, NumberOfRangeGaussianSamples, unsigned long
  );
  itkGetConcreteStrategyMacro(
    Smooth, BilateralType,
    Bilateral, NumberOfRangeGaussianSamples, unsigned long
  );

  /** Get/set CurvatureFlow filter parameters */
  itkSetConcreteStrategyMacro(
    Smooth, CurvatureFlowType,
    CurvatureFlow, TimeStep, typename CurvatureFlowType::TimeStepType
  );
  itkGetConcreteStrategyMacro(
    Smooth, CurvatureFlowType,
    CurvatureFlow, TimeStep, typename CurvatureFlowType::TimeStepType
  );
  itkSetConcreteStrategyMacro(
    Smooth, CurvatureFlowType,
    CurvatureFlow, NumberOfIterations, unsigned int
  );
  itkGetConcreteStrategyMacro(
    Smooth, CurvatureFlowType,
    CurvatureFlow, NumberOfIterations, unsigned int
  );
  itkSetConcreteStrategyMacro(
    Smooth, CurvatureFlowType,
    CurvatureFlow, UseImageSpacing, bool
  );
  itkGetConcreteStrategyMacro(
    Smooth, CurvatureFlowType,
    CurvatureFlow, UseImageSpacing, bool
  );

  /** Get/set DiscreteGaussian filter parameters */
  itkSetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, Variance, typename DiscreteGaussianType::ArrayType
  );
  itkGetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, Variance, typename DiscreteGaussianType::ArrayType
  );
  itkSetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, MaximumError, typename DiscreteGaussianType::ArrayType
  );
  itkGetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, MaximumError, typename DiscreteGaussianType::ArrayType
  );
  itkSetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, MaximumKernelWidth, int
  );
  itkGetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, MaximumKernelWidth, int
  );
  itkSetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, FilterDimensionality, unsigned int
  );
  itkGetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, FilterDimensionality, unsigned int
  );
  itkSetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, UseImageSpacing, bool
  );
  itkGetConcreteStrategyMacro(
    Smooth, DiscreteGaussianType,
    DiscreteGaussian, UseImageSpacing, bool
  );

  /** Get/set RecursiveGaussian filter parameters */
  itkSetConcreteStrategyMacro(
    Smooth, RecursiveGaussianType,
    RecursiveGaussian, Sigma, typename RecursiveGaussianType::ScalarRealType
  );
  itkGetConcreteStrategyMacro(
    Smooth, RecursiveGaussianType,
    RecursiveGaussian, Sigma, typename RecursiveGaussianType::ScalarRealType
  );
  itkSetConcreteStrategyMacro(
    Smooth, RecursiveGaussianType,
    RecursiveGaussian, NormalizeAcrossScale, bool
  );
  itkGetConcreteStrategyMacro(
    Smooth, RecursiveGaussianType,
    RecursiveGaussian, NormalizeAcrossScale, bool
  );

  /** Get/set Median filter parameters */
  itkSetConcreteStrategyMacro(
    Smooth, MedianType,
    Median, Radius, typename MedianType::InputSizeType
  );
  itkGetConcreteStrategyMacro(
    Smooth, MedianType,
    Median, Radius, typename MedianType::InputSizeType
  );

protected:
  SimpleSmoothImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1);
    m_StrategyFactory.template AddStrategy<BilateralType>
      ( SmoothStrategyBilateral );
    m_StrategyFactory.template AddStrategy<CurvatureFlowType>
      ( SmoothStrategyCurvatureFlow );
    m_StrategyFactory.template AddStrategy<DiscreteGaussianType>
      ( SmoothStrategyDiscreteGaussian );
    m_StrategyFactory.template AddStrategy<RecursiveGaussianType>
      ( SmoothStrategyRecursiveGaussian );
    m_StrategyFactory.template AddStrategy<MedianType>
      ( SmoothStrategyMedian );
  }
  virtual ~SimpleSmoothImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleSmoothImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
