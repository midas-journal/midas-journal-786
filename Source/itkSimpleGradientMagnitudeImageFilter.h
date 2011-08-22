/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleGradientMagnitudeImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleGradientMagnitudeImageFilter_h
#define __itkSimpleGradientMagnitudeImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkStrategyFactory.h"
#include "itkFlatStructuringElement.h"

#include "itkGradientMagnitudeImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkMorphologicalGradientImageFilter.h"
#include "itkLaplacianRecursiveGaussianImageFilter.h"

// Force disabling of concept checking
#undef ITK_USE_CONCEPT_CHECKING
#undef itkConceptConstraintsMacro
#undef itkConceptMacro
#define itkConceptConstraintsMacro()
#define itkConceptMacro(name, concept) enum { name = 0 }

#include "itkLaplacianImageFilter.h"

// TODO: Re-enable concept checking

namespace itk
{

/** List of strategies for gradient magnitude image filtering. */
enum GradientMagnitudeStrategy {
  GradientMagnitudeStrategyFiniteDifference,
  GradientMagnitudeStrategyRecursiveGaussian,
  GradientMagnitudeStrategyMorphological,
  GradientMagnitudeStrategyLaplacian,
  GradientMagnitudeStrategyLaplacianRecursiveGaussian
};

/** \class SimpleGradientMagnitudeImageFilter
 * \brief Implements various gradient magnitude image filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   ToutputImage: Optional output image type
 *   TKernel: Optional structuring element for morphological strategy
 *
 * This filter expects the following inputs:
 *   Input1: The image to compute the gradient magnitude
 *
 * This filter produces the following outputs:
 *   Output1: The resultant gradient magnitude image
 *
 * This filter currently supports the following strategies:
 *   FiniteDifference: Computes gradient magnitude using finite difference method
 *   RecursiveGaussian: Computes gradient magnitude using recursive method (see [1])
 *   Morphological: Computes the gradient magnitude as the difference between the
 *     dilation and erosion by the structuring element (see [2])
 *   Laplacian: Computes the 2nd spatial derivative (knonwn as the  Laplacian)
 *   LaplacianRecursiveGaussian: Computes the Laplacian using recursive method (see [1])
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (double) Sigma RecursiveGaussian/LaplacianRecursiveGaussian
 *     Specifies the sigma of the gaussian kernel in world coordinates.
 *     The default is 1.0.
 *   (bool) NormalizeAcrossScale RecursiveGaussian/LaplacianRecursiveGaussian
 *     Specifies flag for normalizing the gaussian over scale space.
 *     True means larger sigmas will not cause the image to fade away,
 *     false means the integral of the image intensity will be kept constant.
 *     The default is false.
 *   (KernelType) Kernel Morphological
 *     The structuring element for mophological gradient magnitude.
 *   (bool) UseImageSpacing Laplacian
 *     Specifies whether image spacing is taken into account when computing the
 *     gradient magnitude.
 *
 * References:
 * [1] Deriche, "Recursively Implementing The Gaussian and Its Derivatives",
 *     INRIA, 1993, ftp://ftp.inria.fr/INRIA/tech-reports/RR/RR-1893.ps.gz
 * [2] Soille, "Morphological Image Analysis: Principles and Applications",
 *     Second Edition, Springer, 2003, Chapter 3.8.1
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage,
          class TOutputImage=TInputImage,
          class TKernel=FlatStructuringElement< ::itk::GetImageDimension<TInputImage>::ImageDimension > >
class ITK_EXPORT SimpleGradientMagnitudeImageFilter :
  public ImageToImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleGradientMagnitudeImageFilter            Self;
  typedef ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleGradientMagnitudeImageFilter, ImageToImageFilter);

  /** Some typedefs. */
  typedef TInputImage                              InputImageType;
  typedef typename    InputImageType::ConstPointer InputImagePointer;
  typedef typename    InputImageType::RegionType   InputRegionType;
  typedef typename    InputImageType::PixelType    InputPixelType;
  typedef TOutputImage                             OutputImageType;
  typedef typename     OutputImageType::Pointer    OutputImagePointer;
  typedef typename     OutputImageType::RegionType OutputRegionType;
  typedef typename     OutputImageType::PixelType  OutputPixelType;
  typedef TKernel                                  KernelType;

  /** Strategy factory typedefs */
  typedef GradientMagnitudeStrategy StrategyKeyType;
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

  /** Get/set RecursiveGaussian filter parameters */
  itkSetConcreteStrategyMacro(
    GradientMagnitude, RecursiveGaussianType,
    RecursiveGaussian, Sigma, double
  );
  itkGetConcreteStrategyMacro(
    GradientMagnitude, RecursiveGaussianType,
    RecursiveGaussian, Sigma, double
  );
  itkSetConcreteStrategyMacro(
    GradientMagnitude, RecursiveGaussianType,
    RecursiveGaussian, NormalizeAcrossScale, bool
  );
  itkGetConcreteStrategyMacro(
    GradientMagnitude, RecursiveGaussianType,
    RecursiveGaussian, NormalizeAcrossScale, bool
  );

  /** Get/set Morphological filter parameters */
  itkSetConcreteStrategyMacro(
    GradientMagnitude, MorphologicalType,
    Morphological, Kernel, KernelType
  );
  itkGetConcreteStrategyMacro(
    GradientMagnitude, MorphologicalType,
    Morphological, Kernel, KernelType
  );

  /** Get/set Laplacian filter parameters */
  itkSetConcreteStrategyMacro(
    GradientMagnitude, LaplacianType,
    Laplacian, UseImageSpacing, bool
  );
  itkGetConcreteStrategyMacro(
    GradientMagnitude, LaplacianType,
    Laplacian, UseImageSpacing, bool
  );

  /** Get/set LaplacianRecursiveGaussian filter parameters */
  itkSetConcreteStrategyMacro(
    GradientMagnitude, LaplacianRecursiveGaussianType,
    LaplacianRecursiveGaussian, Sigma, double
  );
  //itkGetConcreteStrategyMacro(
  //  GradientMagnitude, LaplacianRecursiveGaussianType,
  //  LaplacianRecursiveGaussian, Sigma, double
  //);
  itkSetConcreteStrategyMacro(
    GradientMagnitude, LaplacianRecursiveGaussianType,
    LaplacianRecursiveGaussian, NormalizeAcrossScale, bool
  );
  itkGetConcreteStrategyMacro(
    GradientMagnitude, LaplacianRecursiveGaussianType,
    LaplacianRecursiveGaussian, NormalizeAcrossScale, bool
  );

protected:
  SimpleGradientMagnitudeImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1);
    m_StrategyFactory.template AddStrategy<FiniteDifferenceType>
      ( GradientMagnitudeStrategyFiniteDifference );
    m_StrategyFactory.template AddStrategy<RecursiveGaussianType>
      ( GradientMagnitudeStrategyRecursiveGaussian );
    m_StrategyFactory.template AddStrategy<MorphologicalType>
      ( GradientMagnitudeStrategyMorphological );
    m_StrategyFactory.template AddStrategy<LaplacianType>
      ( GradientMagnitudeStrategyLaplacian );
    m_StrategyFactory.template AddStrategy<LaplacianRecursiveGaussianType>
      ( GradientMagnitudeStrategyLaplacianRecursiveGaussian );
  }
  virtual ~SimpleGradientMagnitudeImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleGradientMagnitudeImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef GradientMagnitudeImageFilter<InputImageType,OutputImageType>
    FiniteDifferenceType;
  typedef GradientMagnitudeRecursiveGaussianImageFilter<InputImageType,OutputImageType>
    RecursiveGaussianType;
  typedef MorphologicalGradientImageFilter<InputImageType,OutputImageType,KernelType>
    MorphologicalType;
  typedef LaplacianImageFilter<InputImageType,OutputImageType>
    LaplacianType;
  typedef LaplacianRecursiveGaussianImageFilter<InputImageType,OutputImageType>
    LaplacianRecursiveGaussianType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
