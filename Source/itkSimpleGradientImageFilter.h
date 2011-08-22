/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleGradientImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleGradientImageFilter_h
#define __itkSimpleGradientImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkStrategyFactory.h"

#include "itkGradientImageFilter.h"
#include "itkGradientRecursiveGaussianImageFilter.h"

namespace itk
{

/** List of strategies for gradient image filtering. */
enum GradientStrategy {
  GradientStrategyFiniteDifference,
  GradientStrategyRecursiveGaussian
};

/** \class SimpleGradientImageFilter
 * \brief Implements various gradient image filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   TCoordRep: Optional coordinate representation type
 *
 * This filter expects the following inputs:
 *   Input1: The scalar image to compute the gradient
 *
 * This filter produces the following outputs:
 *   Output1: The resultant covariant vector gradient image
 *
 * This filter currently supports the following strategies:
 *   FiniteDifference: Computes gradient using finite difference method
 *   RecursiveGaussian: Computes gradient using recursive method (see [1])
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (double) Sigma RecursiveGaussian
 *     Specifies the sigma of the gaussian kernel in world coordinates.
 *     The default is 1.0.
 *   (bool) NormalizeAcrossScale RecursiveGaussian
 *     Specifies flag for normalizing the gaussian over scale space.
 *     True means larger sigmas will not cause the image to fade away,
 *     false means the integral of the image intensity will be kept constant.
 *     The default is false.
 *   (bool) UseImageDirection RecursiveGaussian
 *     Flag specifying whether gradient is computed on image grid or with
 *     respect to the physical space.
 *
 * References:
 * [1] Deriche, "Recursively Implementing The Gaussian and Its Derivatives",
 *     INRIA, 1993, ftp://ftp.inria.fr/INRIA/tech-reports/RR/RR-1893.ps.gz
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage, typename TCoordRep=float>
class ITK_EXPORT SimpleGradientImageFilter :
  public ImageToImageFilter<TInputImage,
                            Image<CovariantVector<TCoordRep, ::itk::GetImageDimension<TInputImage>::ImageDimension>, ::itk::GetImageDimension<TInputImage>::ImageDimension> >
{
public:
  /** Standard class typedefs. */
  typedef SimpleGradientImageFilter                     Self;
  typedef ImageToImageFilter<TInputImage,Image<CovariantVector<TCoordRep, ::itk::GetImageDimension<TInputImage>::ImageDimension>, ::itk::GetImageDimension<TInputImage>::ImageDimension> > Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleGradientImageFilter, ImageToImageFilter);

  /** Some typedefs. */
  typedef TInputImage                           InputImageType;
  typedef typename InputImageType::ConstPointer InputImagePointer;
  typedef typename InputImageType::RegionType   InputRegionType;
  typedef typename InputImageType::PixelType    InputPixelType;
  typedef typename Superclass::OutputImageType  OutputImageType;
  typedef typename OutputImageType::Pointer     OutputImagePointer;
  typedef typename OutputImageType::RegionType  OutputRegionType;
  typedef typename OutputImageType::PixelType   OutputPixelType;
  typedef TCoordRep                             CoordRepType;

  /** Strategy factory typedefs */
  typedef GradientStrategy StrategyKeyType;
  typedef Superclass StrategyType;
  typedef StrategyFactory<StrategyKeyType,StrategyType> StrategyFactoryType;

  /** Get/set the strategy */
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
    Gradient, RecursiveGaussianType,
    RecursiveGaussian, Sigma, double
  );
  //itkGetConcreteStrategyMacro(
  //  Gradient, RecursiveGaussianType,
  //  RecursiveGaussian, Sigma, double
  //);
  itkSetConcreteStrategyMacro(
    Gradient, RecursiveGaussianType,
    RecursiveGaussian, NormalizeAcrossScale, bool
  );
  itkGetConcreteStrategyMacro(
    Gradient, RecursiveGaussianType,
    RecursiveGaussian, NormalizeAcrossScale, bool
  );
  itkSetConcreteStrategyMacro(
    Gradient, RecursiveGaussianType,
    RecursiveGaussian, UseImageDirection, bool
  );
  itkGetConcreteStrategyMacro(
    Gradient, RecursiveGaussianType,
    RecursiveGaussian, UseImageDirection, bool
  );

protected:
  SimpleGradientImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1);
    m_StrategyFactory.template AddStrategy<FiniteDifferenceType>
      ( GradientStrategyFiniteDifference );
    m_StrategyFactory.template AddStrategy<RecursiveGaussianType>
      ( GradientStrategyRecursiveGaussian );
  }
  virtual ~SimpleGradientImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleGradientImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef GradientImageFilter<InputImageType, CoordRepType, CoordRepType>
    FiniteDifferenceType;
  typedef GradientRecursiveGaussianImageFilter<InputImageType,OutputImageType>
    RecursiveGaussianType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
