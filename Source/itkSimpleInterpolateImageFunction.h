/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleInterpolateImageFunction.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleInterpolateImageFunction_h
#define __itkSimpleInterpolateImageFunction_h

#include "itkInterpolateImageFunction.h"
#include "itkStrategyFactory.h"

#include "itkNearestNeighborInterpolateImageFunction.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkBSplineInterpolateImageFunction.h"
#include "itkWindowedSincInterpolateImageFunction.h"

namespace itk
{

/** List of strategies for image interpolation functions. */
enum InterpolateImageFunctionStrategy {
  InterpolateImageFunctionStrategyNearestNeighbor,
  InterpolateImageFunctionStrategyLinear,
  InterpolateImageFunctionStrategyBSpline,
  InterpolateImageFunctionStrategyBlackmanWindowedSinc,
  InterpolateImageFunctionStrategyCosineWindowedSinc,
  InterpolateImageFunctionStrategyHammingWindowedSinc,
  InterpolateImageFunctionStrategyLanczosWindowedSinc,
  InterpolateImageFunctionStrategyWelchWindowedSinc
};

/** \class SimpleInterpolateImageFunction
 * \brief Implements various image interpolation functions.
 *
 * This function is templated over the following types:
 *   TInputImage: Required input image type
 *   TCoordRep: Optional coordinate representation type
 *   TBoundaryCondition: Optional boundary type
 *
 * This function expects the following inputs:
 *   Input1: The image to evaluate with the interpolation function
 *
 * This function currently supports the following strategies:
 *   NearestNeighbor: Evaluates function at non-integer pixel position by
 *     returning the intensity at the nearest neighbor.
 *   Linear: Evaluates function at non-integer pixel position by computing
 *     linear combination of surrounding pixels.
 *   BSpline: Evaluates function at non-integer pixel positions by computing
 *     the non-linear combination of surrounding pixels using B-splines.
 *     See [1], [2], and [3].
 *   BlackmanWindowedSinc: Evaluates function at non-integer pixel positions
 *     using windowed sinc with Blackman window. See [4].
 *   CosineWindowedSinc: Evaluates function at non-integer pixel positions
 *     using windowed sinc with Cosine window. See [4].
 *   HammingWindowedSinc: Evaluates function at non-integer pixel positions
 *     using windowed sinc with Hamming window. See [4].
 *   LanczosWindowedSinc: Evaluates function at non-integer pixel positions
 *     using windowed sinc with Lanczos window. See [4].
 *   WelchWindowedSinc: Evaluates function at non-integer pixel positions
 *     using windowed sinc with Welch window. See [4].
 *
 * References:
 * [1] Unser, "Splines: A Perfect Fit for Signal and Image Processing,"
 *     IEEE Signal Processing Magazine, vol. 16, no. 6, pp. 22--38, 1999.
 * [2] Unser, Aldroubi and Eden, "B-Spline Signal Processing: Part I--Theory,"
 *     IEEE Transactions on Signal Processing, vol.41, no.2, 1993,
 *     pp. 821--832.
 * [3] Unser, Aldroubi and Eden, "B-Spline Signal Processing: Part II--
 *     Efficient Design and Applications," IEEE Transactions on Signal
 *     Processing, vol. 41, no. 2, 1993, pp. 834--848.
 * [4] Meijering, Niessen, Pluim, Viergever, "Quantitative Comparison
 *     of Sinc-Approximating Kernels for Medical Image Interpolation",
 *     MICCAI, 1999, pp. 210--217
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage,
          class TCoordRep=double,
          unsigned int VRadius=3,
          class TBoundaryCondition=ConstantBoundaryCondition<TInputImage> >
class ITK_EXPORT SimpleInterpolateImageFunction :
  public InterpolateImageFunction<TInputImage,TCoordRep>
{
public:
  /** Standard class typedefs. */
  typedef SimpleInterpolateImageFunction                  Self;
  typedef InterpolateImageFunction<TInputImage,TCoordRep> Superclass;
  typedef SmartPointer<Self>                              Pointer;
  typedef SmartPointer<const Self>                        ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleInterpolateImageFunction, InterpolateImageFunction);

  /** Some typedefs. */
  typedef typename Superclass::InputImageType InputImageType;
  typedef typename Superclass::InputPixelType InputPixelType;
  typedef typename Superclass::CoordRepType CoordRepType;
  typedef typename Superclass::OutputType OutputType;
  typedef typename Superclass::RealType RealType;
  typedef typename Superclass::PointType PointType;
  typedef typename Superclass::IndexType IndexType;
  typedef typename Superclass::IndexValueType IndexValueType;
  typedef typename Superclass::ContinuousIndexType ContinuousIndexType;
  itkStaticConstMacro(ImageDimension, unsigned int,Superclass::ImageDimension);

  /** Strategy factory typedefs */
  typedef InterpolateImageFunctionStrategy StrategyKeyType;
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

  /** Set the input image. */
  virtual void SetInputImage( const InputImageType * ptr )
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    function->SetInputImage(ptr);
    }

  /** Get the input image. */
  const InputImageType* GetInputImage() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->GetInputImage();
    }

  /** Evaluate the function at specified Point position. */
  virtual OutputType Evaluate(const PointType& point) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->Evaluate(point);
    }

  /** Evaluate the function at specified Index position. */
  virtual OutputType EvaluateAtIndex(const IndexType& index) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->EvaluateAtIndex(index);
    }

  /** Evaluate the function at specified ContinuousIndex position. */
  virtual OutputType EvaluateAtContinuousIndex(const ContinuousIndexType& index) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->EvaluateAtContinuousIndex(index);
    }

  virtual bool IsInsideBuffer(const IndexType& index) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->IsInsideBuffer(index);
    }

  virtual bool IsInsideBuffer(const ContinuousIndexType& index) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->IsInsideBuffer(index);
    }

  virtual bool IsInsideBuffer(const PointType& point) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->IsInsideBuffer(point);
    }

  void ConvertPointToNearestIndex(const PointType& point, IndexType& index) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    function->ConvertPointToNearestIndex(point, index);
    }

  void ConvertPointToContinuousIndex(const PointType& point, ContinuousIndexType & cindex) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    function->ConvertPointToContinuousIndex(point, cindex);
    }

  inline void ConvertContinuousIndexToNearestIndex(
    const ContinuousIndexType& cindex, IndexType& index) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    function->ConvertContinuousIndexToNearestIndex(cindex, index);
    }

  virtual const IndexType& GetStartIndex() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->GetStartIndex();
    }

  virtual const IndexType& GetEndIndex() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->GetEndIndex();
    }

  virtual const ContinuousIndexType& GetStartContinuousIndex() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->GetStartContinuousIndex();
    }

  virtual const ContinuousIndexType& GetEndContinuousIndex() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* function = ptrStrategyFactory->GetStrategy();
    return function->GetEndContinuousIndex();
    }

protected:
  SimpleInterpolateImageFunction() : m_StrategyFactory(StrategyFactoryType(this))
    {
    m_StrategyFactory.template AddStrategy<NearestNeighborInterpolateType>
      ( InterpolateImageFunctionStrategyNearestNeighbor );
    m_StrategyFactory.template AddStrategy<LinearInterpolateType>
      ( InterpolateImageFunctionStrategyLinear );
    m_StrategyFactory.template AddStrategy<BSplineInterpolateType>
      ( InterpolateImageFunctionStrategyBSpline );
    m_StrategyFactory.template AddStrategy<BlackmanWindowedSincInterpolateType>
      ( InterpolateImageFunctionStrategyBlackmanWindowedSinc );
    m_StrategyFactory.template AddStrategy<CosineWindowedSincInterpolateType>
      ( InterpolateImageFunctionStrategyCosineWindowedSinc );
    m_StrategyFactory.template AddStrategy<HammingWindowedSincInterpolateType>
      ( InterpolateImageFunctionStrategyHammingWindowedSinc );
    m_StrategyFactory.template AddStrategy<LanczosWindowedSincInterpolateType>
      ( InterpolateImageFunctionStrategyLanczosWindowedSinc );
    m_StrategyFactory.template AddStrategy<WelchWindowedSincInterpolateType>
      ( InterpolateImageFunctionStrategyWelchWindowedSinc );
    }
  virtual ~SimpleInterpolateImageFunction() {};

private:
  SimpleInterpolateImageFunction(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef NearestNeighborInterpolateImageFunction
    <InputImageType,CoordRepType> NearestNeighborInterpolateType;
  typedef LinearInterpolateImageFunction
    <InputImageType,CoordRepType> LinearInterpolateType;
  typedef BSplineInterpolateImageFunction
    <InputImageType,CoordRepType> BSplineInterpolateType;
  typedef WindowedSincInterpolateImageFunction
    <InputImageType,VRadius,Function::BlackmanWindowFunction<VRadius>,TBoundaryCondition,CoordRepType>
    BlackmanWindowedSincInterpolateType;
  typedef WindowedSincInterpolateImageFunction
    <InputImageType,VRadius,Function::CosineWindowFunction<VRadius>,TBoundaryCondition,CoordRepType>
    CosineWindowedSincInterpolateType;
  typedef WindowedSincInterpolateImageFunction
    <InputImageType,VRadius,Function::HammingWindowFunction<VRadius>,TBoundaryCondition,CoordRepType>
    HammingWindowedSincInterpolateType;
  typedef WindowedSincInterpolateImageFunction
    <InputImageType,VRadius,Function::LanczosWindowFunction<VRadius>,TBoundaryCondition,CoordRepType>
    LanczosWindowedSincInterpolateType;
  typedef WindowedSincInterpolateImageFunction
    <InputImageType,VRadius,Function::WelchWindowFunction<VRadius>,TBoundaryCondition,CoordRepType>
    WelchWindowedSincInterpolateType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
