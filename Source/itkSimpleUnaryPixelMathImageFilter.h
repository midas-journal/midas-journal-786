/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleUnaryPixelMathImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleUnaryPixelMathImageFilter_h
#define __itkSimpleUnaryPixelMathImageFilter_h

#include "itkNumericTraits.h"
#include "itkInPlaceImageFilter.h"
#include "itkStrategyFactory.h"

#include "itkAbsImageFilter.h"
#include "itkAcosImageFilter.h"
#include "itkAsinImageFilter.h"
#include "itkAtanImageFilter.h"
#include "itkBoundedReciprocalImageFilter.h"
#include "itkCosImageFilter.h"
#include "itkExpImageFilter.h"
#include "itkExpNegativeImageFilter.h"
#include "itkInvertIntensityImageFilter.h"
#include "itkLog10ImageFilter.h"
#include "itkLogImageFilter.h"
#include "itkNotImageFilter.h"
#include "itkSigmoidImageFilter.h"
#include "itkSinImageFilter.h"
#include "itkSqrtImageFilter.h"
#include "itkSquareImageFilter.h"
#include "itkTanImageFilter.h"
/*
// Review filters
#include "itkAddConstantImageFilter.h"
#include "itkDivideByConstantImageFilter.h"
#include "itkMultiplyByConstantImageFilter.h"
#include "itkSubtractConstantImageFilter.h"
*/
// TODO: ShiftScale

namespace itk
{

/** List of strategies for unary image filtering. */
enum UnaryPixelMathStrategy {
  UnaryPixelMathStrategyAbs,
  UnaryPixelMathStrategyAcos,
  UnaryPixelMathStrategyAsin,
  UnaryPixelMathStrategyAtan,
  UnaryPixelMathStrategyBoundedReciprocal,
  UnaryPixelMathStrategyCos,
  UnaryPixelMathStrategyExp,
  UnaryPixelMathStrategyExpNegative,
  UnaryPixelMathStrategyInvertIntensity,
  UnaryPixelMathStrategyLog10,
  UnaryPixelMathStrategyLog,
  UnaryPixelMathStrategyNot,
  UnaryPixelMathStrategySigmoid,
  UnaryPixelMathStrategySin,
  UnaryPixelMathStrategySqrt,
  UnaryPixelMathStrategySquare,
  UnaryPixelMathStrategyTan,
  /*
  // Review filters
  UnaryPixelMathStrategyAddConstant,
  UnaryPixelMathStrategyDivideByConstant,
  UnaryPixelMathStrategyMultiplyByConstant,
  UnaryPixelMathStrategySubtractConstant
  */
};

/** \class SimpleUnaryPixelMathImageFilter
 * \brief Implements various unary function image filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   ToutputImage: Optional output image type
 *
 * This filter expects the following inputs:
 *   Input1: The image to apply the unary function operation
 *
 * This filter produces the following outputs:
 *   Output1: The resultant image
 *
 * This filter currently supports the following strategies:
 *   Abs: out = abs(in)
 *   Acos: out = acos(in)
 *   Asin: out = asin(in)
 *   Atan: out = atan(in)
 *   BoundedReciprocal: out = 1/(1+in)
 *   Cos: out = cos(in)
 *   Exp: out = exp(in)
 *   ExpNegative: out = exp(-factor*in)
 *   InvertIntensity: out = max-in
 *   Log10: out = log_10(in)
 *   Log: out = log(in)
 *   Not: out = !in
 *   Sigmoid: out = (max-min) * ( 1 / ( 1 + exp((x-beta)/alpha)) ) + min )
 *   Sin: out = sin(in)
 *   Sqrt: out = sqrt(in)
 *   Square: out = in*in
 *   Tan: out = tan(in)
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (double) Factor ExpNegative
 *     Specifies the factor.
 *   (InputPixelType) Maximum InvertIntensity
 *     Specifies the maximum value.
 *   (double) Alpha Sigmoid
 *     Specifies the alpha value.
 *   (double) Beta Sigmoid
 *     Specifies the beta value.
 *   (OutputPixelType) OutputMinimum Sigmoid
 *     Specifies the output minimum value.
 *   (OutputPixelType) OutputMaximum Sigmoid
 *     Specifies the output maximum value.
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage, class TOutputImage=TInputImage>
class ITK_EXPORT SimpleUnaryPixelMathImageFilter :
  public InPlaceImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleUnaryPixelMathImageFilter               Self;
  typedef InPlaceImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleUnaryPixelMathImageFilter, InPlaceImageFilter);

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
  typedef UnaryPixelMathStrategy StrategyKeyType;
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

  /** Get/set ExpNegative flter parameters */
  itkSetConcreteStrategyMacro(
    UnaryPixelMath, ExpNegativeImageFilterType,
    ExpNegative, Factor, double
  );
  //itkGetConcreteStrategyMacro(
  //  UnaryPixelMath, ExpNegativeImageFilterType,
  //  ExpNegative, Factor, double
  //);

  /** Get/set InvertIntensity filter parameters */
  itkSetConcreteStrategyMacro(
    UnaryPixelMath, InvertIntensityImageFilterType,
    InvertIntensity, Maximum, InputPixelType
  );
  itkGetConcreteStrategyMacro(
    UnaryPixelMath, InvertIntensityImageFilterType,
    InvertIntensity, Maximum, InputPixelType
  );

  /** Get/set Sigmoid filter parameters */
  itkSetConcreteStrategyMacro(
    UnaryPixelMath, SigmoidImageFilterType,
    Sigmoid, Alpha, double
  );
  itkSetConcreteStrategyMacro(
    UnaryPixelMath, SigmoidImageFilterType,
    Sigmoid, Beta, double
  );
  itkSetConcreteStrategyMacro(
    UnaryPixelMath, SigmoidImageFilterType,
    Sigmoid, OutputMinimum, OutputPixelType
  );
  itkSetConcreteStrategyMacro(
    UnaryPixelMath, SigmoidImageFilterType,
    Sigmoid, OutputMaximum, OutputPixelType
  );

protected:
  SimpleUnaryPixelMathImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1);
    this->InPlaceOn();
    m_StrategyFactory.template AddStrategy<AbsImageFilterType>
      ( UnaryPixelMathStrategyAbs );
    m_StrategyFactory.template AddStrategy<AcosImageFilterType>
      ( UnaryPixelMathStrategyAcos );
    m_StrategyFactory.template AddStrategy<AsinImageFilterType>
      ( UnaryPixelMathStrategyAsin );
    m_StrategyFactory.template AddStrategy<AtanImageFilterType>
      ( UnaryPixelMathStrategyAtan );
    m_StrategyFactory.template AddStrategy<BoundedReciprocalImageFilterType>
      ( UnaryPixelMathStrategyBoundedReciprocal );
    m_StrategyFactory.template AddStrategy<CosImageFilterType>
      ( UnaryPixelMathStrategyCos );
    m_StrategyFactory.template AddStrategy<ExpImageFilterType>
      ( UnaryPixelMathStrategyExp );
    m_StrategyFactory.template AddStrategy<ExpNegativeImageFilterType>
      ( UnaryPixelMathStrategyExpNegative );
    m_StrategyFactory.template AddStrategy<InvertIntensityImageFilterType>
      ( UnaryPixelMathStrategyInvertIntensity );
    m_StrategyFactory.template AddStrategy<Log10ImageFilterType>
      ( UnaryPixelMathStrategyLog10 );
    m_StrategyFactory.template AddStrategy<LogImageFilterType>
      ( UnaryPixelMathStrategyLog );
    m_StrategyFactory.template AddStrategy<NotImageFilterType>
      ( UnaryPixelMathStrategyNot );
    m_StrategyFactory.template AddStrategy<SigmoidImageFilterType>
      ( UnaryPixelMathStrategySigmoid );
    m_StrategyFactory.template AddStrategy<SinImageFilterType>
      ( UnaryPixelMathStrategySin );
    m_StrategyFactory.template AddStrategy<SqrtImageFilterType>
      ( UnaryPixelMathStrategySqrt );
    m_StrategyFactory.template AddStrategy<SquareImageFilterType>
      ( UnaryPixelMathStrategySquare );
    m_StrategyFactory.template AddStrategy<TanImageFilterType>
      ( UnaryPixelMathStrategyTan );
    /*
    // Review filters
    m_StrategyFactory.template AddStrategy<AddConstantImageFilterType>( UnaryPixelMathStrategyAddConstant );
    m_StrategyFactory.template AddStrategy<DivideByConstantImageFilterType>( UnaryPixelMathStrategyDivideByConstant );
    m_StrategyFactory.template AddStrategy<MultiplyByConstantImageFilterType>( UnaryPixelMathStrategyMultiplyByConstant );
    m_StrategyFactory.template AddStrategy<SubtractConstantImageFilterType>( UnaryPixelMathStrategySubtractConstant );
    */
  }
  virtual ~SimpleUnaryPixelMathImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleUnaryPixelMathImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef AbsImageFilter<InputImageType,OutputImageType>
    AbsImageFilterType;
  typedef AcosImageFilter<InputImageType,OutputImageType>
    AcosImageFilterType;
  typedef AsinImageFilter<InputImageType,OutputImageType>
    AsinImageFilterType;
  typedef AtanImageFilter<InputImageType,OutputImageType>
    AtanImageFilterType;
  typedef BoundedReciprocalImageFilter<InputImageType,OutputImageType>
    BoundedReciprocalImageFilterType;
  typedef CosImageFilter<InputImageType,OutputImageType>
    CosImageFilterType;
  typedef ExpImageFilter<InputImageType,OutputImageType>
    ExpImageFilterType;
  typedef ExpNegativeImageFilter<InputImageType,OutputImageType>
    ExpNegativeImageFilterType;
  typedef InvertIntensityImageFilter<InputImageType,OutputImageType>
    InvertIntensityImageFilterType;
  typedef Log10ImageFilter<InputImageType,OutputImageType>
    Log10ImageFilterType;
  typedef LogImageFilter<InputImageType,OutputImageType>
    LogImageFilterType;
  typedef NotImageFilter<InputImageType,OutputImageType>
    NotImageFilterType;
  typedef SigmoidImageFilter<InputImageType,OutputImageType>
    SigmoidImageFilterType;
  typedef SinImageFilter<InputImageType,OutputImageType>
    SinImageFilterType;
  typedef SqrtImageFilter<InputImageType,OutputImageType>
    SqrtImageFilterType;
  typedef SquareImageFilter<InputImageType,OutputImageType>
    SquareImageFilterType;
  typedef TanImageFilter<InputImageType,OutputImageType>
    TanImageFilterType;
  /*
  // Review filters
  typedef AddConstantImageFilter<InputImageType,OutputImageType> AddConstantImageFilterType;
  typedef DivideByConstantImageFilter<InputImageType,OutputImageType> DivideByConstantImageFilterType;
  typedef MultiplyByConstantImageFilter<InputImageType,OutputImageType> MultiplyByConstantImageFilterType;
  typedef SubtractConstantImageFilter<InputImageType,OutputImageType> SubtractConstantImageFilterType;
  */

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
