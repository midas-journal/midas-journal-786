/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleBinaryPixelMathImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleBinaryPixelMathImageFilter_h
#define __itkSimpleBinaryPixelMathImageFilter_h

#include "itkNumericTraits.h"
#include "itkInPlaceImageFilter.h"
#include "itkStrategyFactory.h"

//#include "itkAndImageFilter.h"
//#include "itkOrImageFilter.h"
//#include "itkXorImageFilter.h"
#include "itkAddImageFilter.h"
#include "itkSubtractImageFilter.h"
#include "itkMultiplyImageFilter.h"
#include "itkDivideImageFilter.h"
#include "itkWeightedAddImageFilter.h"
#include "itkAbsoluteValueDifferenceImageFilter.h"
#include "itkBinaryMagnitudeImageFilter.h"
#include "itkConstrainedValueAdditionImageFilter.h"
#include "itkConstrainedValueDifferenceImageFilter.h"
#include "itkMaskImageFilter.h"
#include "itkMaskNegatedImageFilter.h"
#include "itkMaximumImageFilter.h"
#include "itkMinimumImageFilter.h"
#include "itkSquaredDifferenceImageFilter.h"

namespace itk
{

/** List of strategies for binary function operations. */
enum BinaryPixelMathStrategy {
  //BinaryPixelMathStrategyAnd,
  //BinaryPixelMathStrategyOr,
  //BinaryPixelMathStrategyXor,
  BinaryPixelMathStrategyAdd,
  BinaryPixelMathStrategySubtract,
  BinaryPixelMathStrategyMultiply,
  BinaryPixelMathStrategyDivide,
  BinaryPixelMathStrategyWeightedAdd,
  BinaryPixelMathStrategyAbsoluteValueDifference,
  BinaryPixelMathStrategyBinaryMagnitude,
  BinaryPixelMathStrategyConstrainedValueAddition,
  BinaryPixelMathStrategyConstrainedValueDifference,
  BinaryPixelMathStrategyMask,
  BinaryPixelMathStrategyMaskNegated,
  BinaryPixelMathStrategyMaximum,
  BinaryPixelMathStrategyMinimum,
  BinaryPixelMathStrategySquaredDifference
};

/** \class SimpleBinaryPixelMathImageFilter
 * \brief Implements various binary function operations.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   TOutputImage: Optional output image type
 *
 * This filter expects the following inputs:
 *   Input1: The first input image
 *   Input2: The second input image
 *
 * This filter produces the following outputs:
 *   Output1: The resultant image after selected binary operation
 *
 * This filter currently supports the following strategies:
 *   Add: out = in1 + in2
 *   Subtract: out = in1 - in2
 *   Multiply: out = in1 * in2
 *   Divide: out = in1 / in2
 *   WeightedAdd: out = Alpha*in1 + (1-Alpha)*in2
 *   AbsoluteValueDifference: out = abs(in1 - in2)
 *   BinaryMagnitude: out = sqrt(in1*in1 + in2*in2)
 *   ConstrainedValueAddition: out = clamp(in1+in2, pixelmin, pixelmax)
 *   ConstrainedValueDifference: out = clamp(in1-in2, pixelmin, pixelmax)
 *   Mask: out = (mask>0) ? in1 : 0
 *   MaskNegated: out = (mask>0) ? 0 : in1
 *   Maximum: out = max(in1, in2)
 *   Minimum  out = min(in1, in2)
 *   SquaredDifference: out = (in1-in2) * (in1-in2)
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (double) Alpha WeightedAdd
 *     Weight value for pixelwise addition.
 *   (OutputPixelType) OutsideValue Mask
 *     Value when pixel is outside of mask. The default is 0.
 *   (OutputPixelType) OutsideValue NegatedMask
 *     Value when pixel is inside of mask. The default is 0.
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage, class TOutputImage=TInputImage>
class ITK_EXPORT SimpleBinaryPixelMathImageFilter :
  public InPlaceImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleBinaryPixelMathImageFilter              Self;
  typedef InPlaceImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleBinaryPixelMathImageFilter, InPlaceImageFilter);

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
  typedef BinaryPixelMathStrategy StrategyKeyType;
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

  /** Connect input 1. */
  void SetInput1(const TInputImage* image1)
    {
    this->SetInput(0, const_cast<TInputImage*>( image1 ));
    }

  /** Connect input 2. */
  void SetInput2(const TInputImage* image2)
    {
    this->SetInput(1, const_cast<TInputImage*>( image2 ));
    }

  /** Set WeightedAdd filter parameters */
  itkSetConcreteStrategyMacro(
    BinaryPixelMath, WeightedAddImageFilterType,
    WeightedAdd, Alpha, double
  );

  /** Get/set Mask filter parameters */
  itkSetConcreteStrategyMacro(
    BinaryPixelMath, MaskImageFilterType,
    Mask, OutsideValue, OutputPixelType
  );
  itkGetConcreteStrategyMacro(
    BinaryPixelMath, MaskImageFilterType,
    Mask, OutsideValue, OutputPixelType
  );

  /** Get/set MaskNegated filter parameters */
  itkSetConcreteStrategyMacro(
    BinaryPixelMath, MaskNegatedImageFilterType,
    MaskNegated, OutsideValue, OutputPixelType
  );
  itkGetConcreteStrategyMacro(
    BinaryPixelMath, MaskNegatedImageFilterType,
    MaskNegated, OutsideValue, OutputPixelType
  );

protected:
  SimpleBinaryPixelMathImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(2);
    this->InPlaceOn();
    //m_StrategyFactory.template AddStrategy<AndImageFilterType>
    //  ( BinaryPixelMathStrategyAnd );
    //m_StrategyFactory.template AddStrategy<OrImageFilterType>
    //  ( BinaryPixelMathStrategyOr );
    //m_StrategyFactory.template AddStrategy<XorImageFilterType>
    //  ( BinaryPixelMathStrategyXor );
    m_StrategyFactory.template AddStrategy<AddImageFilterType>
      ( BinaryPixelMathStrategyAdd );
    m_StrategyFactory.template AddStrategy<SubtractImageFilterType>
      ( BinaryPixelMathStrategySubtract );
    m_StrategyFactory.template AddStrategy<MultiplyImageFilterType>
      ( BinaryPixelMathStrategyMultiply );
    m_StrategyFactory.template AddStrategy<DivideImageFilterType>
      ( BinaryPixelMathStrategyDivide );
    m_StrategyFactory.template AddStrategy<WeightedAddImageFilterType>
      ( BinaryPixelMathStrategyWeightedAdd );
    m_StrategyFactory.template AddStrategy<AbsoluteValueDifferenceImageFilterType>
      ( BinaryPixelMathStrategyAbsoluteValueDifference );
    m_StrategyFactory.template AddStrategy<BinaryMagnitudeImageFilterType>
      ( BinaryPixelMathStrategyBinaryMagnitude );
    m_StrategyFactory.template AddStrategy<ConstrainedValueAdditionImageFilterType>
      ( BinaryPixelMathStrategyConstrainedValueAddition );
    m_StrategyFactory.template AddStrategy<ConstrainedValueDifferenceImageFilterType>
      ( BinaryPixelMathStrategyConstrainedValueDifference );
    m_StrategyFactory.template AddStrategy<MaskImageFilterType>
      ( BinaryPixelMathStrategyMask );
    m_StrategyFactory.template AddStrategy<MaskNegatedImageFilterType>
      ( BinaryPixelMathStrategyMaskNegated );
    m_StrategyFactory.template AddStrategy<MaximumImageFilterType>
      ( BinaryPixelMathStrategyMaximum );
    m_StrategyFactory.template AddStrategy<MinimumImageFilterType>
      ( BinaryPixelMathStrategyMinimum );
    m_StrategyFactory.template AddStrategy<SquaredDifferenceImageFilterType>
      ( BinaryPixelMathStrategySquaredDifference );
  }
  virtual ~SimpleBinaryPixelMathImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( 0, const_cast<TInputImage*>(this->GetInput(0)) );
    filter->SetInput( 1, const_cast<TInputImage*>(this->GetInput(1)) );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleBinaryPixelMathImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  //typedef AndImageFilter<InputImageType,InputImageType,OutputImageType>
  //  AndImageFilterType;
  //typedef OrImageFilter<InputImageType,InputImageType,OutputImageType>
  //  OrImageFilterType;
  //typedef XorImageFilter<InputImageType,InputImageType,OutputImageType>
  //  XorImageFilterType;
  typedef AddImageFilter<InputImageType,InputImageType,OutputImageType>
    AddImageFilterType;
  typedef SubtractImageFilter<InputImageType,InputImageType,OutputImageType>
    SubtractImageFilterType;
  typedef MultiplyImageFilter<InputImageType,InputImageType,OutputImageType>
    MultiplyImageFilterType;
  typedef DivideImageFilter<InputImageType,InputImageType,OutputImageType>
    DivideImageFilterType;
  typedef WeightedAddImageFilter<InputImageType,InputImageType,OutputImageType>
    WeightedAddImageFilterType;
  typedef AbsoluteValueDifferenceImageFilter<InputImageType,InputImageType,OutputImageType>
    AbsoluteValueDifferenceImageFilterType;
  typedef BinaryMagnitudeImageFilter<InputImageType,InputImageType,OutputImageType>
    BinaryMagnitudeImageFilterType;
  typedef ConstrainedValueAdditionImageFilter<InputImageType,InputImageType,OutputImageType>
    ConstrainedValueAdditionImageFilterType;
  typedef ConstrainedValueDifferenceImageFilter<InputImageType,InputImageType,OutputImageType>
    ConstrainedValueDifferenceImageFilterType;
  typedef MaskImageFilter<InputImageType,InputImageType,OutputImageType>
    MaskImageFilterType;
  typedef MaskNegatedImageFilter<InputImageType,InputImageType,OutputImageType>
    MaskNegatedImageFilterType;
  typedef MaximumImageFilter<InputImageType,InputImageType,OutputImageType>
    MaximumImageFilterType;
  typedef MinimumImageFilter<InputImageType,InputImageType,OutputImageType>
    MinimumImageFilterType;
  typedef SquaredDifferenceImageFilter<InputImageType,InputImageType,OutputImageType>
    SquaredDifferenceImageFilterType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
