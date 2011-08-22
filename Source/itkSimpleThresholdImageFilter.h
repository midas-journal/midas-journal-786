/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleThresholdImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleThresholdImageFilter_h
#define __itkSimpleThresholdImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkStrategyFactory.h"

#include "itkBinaryThresholdImageFilter.h"
#include "itkThresholdImageFilter.h"
#include "itkDoubleThresholdImageFilter.h"
#include "itkOtsuThresholdImageFilter.h"
#include "itkKappaSigmaThresholdImageFilter.h"

namespace itk
{

/** List of strategies for threshold image filtering. */
enum ThresholdStrategy {
  ThresholdStrategyBinary,
  ThresholdStrategySingle,
  ThresholdStrategyDouble,
  ThresholdStrategyOtsu,
  ThresholdStrategyKappaSigma
};

/** \class SimpleThresholdImageFilter
 * \brief Implements various threshold image filters.
 *
 * This filter is templated over the following types:
 *   TImage: Required input (and output) image type
 *
 * This filter expects the following inputs:
 *   Input1: The image to apply the threshold operation
 *
 * This filter produces the following outputs:
 *   Output1: The resultant thresholded image
 *
 * This filter currently supports the following strategies:
 *   Binary: manual binary threshold
 *   Single: manual single threshold (passes through inside values)
 *   Double: double threshold operation (narrow and wide range)
 *   Otsu: automatic threshold calculation based on histogram
 *   KappaSigma: automatic threshold calculation based on maximum entropy
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (ImagePixelType) OutsideValue All
 *     Specifies the pixel value when outside the threshold.
 *   (ImagePixelType) OutsideValue Binary/Double/Otsu/KappaSigma
 *     Specifies the pixel value when inside the threshold.
 *   (ImagePixelType) LowerThreshold Binary/Single
 *     Specifies the manual lower threshold value.
 *   (ImagePixelType) UpperThreshold Binary/Single
 *     Specifies the manual upper threshold value.
 *   (ImagePixelType) Threshold1 Double
 *     TODO:
 *   (ImagePixelType) Threshold2 Double
 *     TODO:
 *   (ImagePixelType) Threshold3 Double
 *     TODO:
 *   (ImagePixelType) Threshold4 Double
 *     TODO:
 *   (unsigned long) NumberOfHistogramBins Otsu
 *     TODO:
 *   (MaskImagePixelType) MaskValue KappaSigma
 *     TODO:
 *   (double) SigmaFactor KappaSigma
 *     TODO:
 *   (unsigned int) NumberOfIterations KappaSigma
 *     TODO:
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TImage>
class ITK_EXPORT SimpleThresholdImageFilter :
  public ImageToImageFilter<TImage,TImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleThresholdImageFilter         Self;
  typedef ImageToImageFilter<TImage,TImage>  Superclass;
  typedef SmartPointer<Self>                 Pointer;
  typedef SmartPointer<const Self>           ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleThresholdImageFilter, ImageToImageFilter);

  /** Some typedefs. */
  typedef TImage                              ImageType;
  typedef typename    ImageType::ConstPointer ImagePointer;
  typedef typename    ImageType::RegionType   ImageRegionType;
  typedef typename    ImageType::PixelType    ImagePixelType;
  typedef unsigned char                       MaskImagePixelType;
  typedef Image<MaskImagePixelType,::itk::GetImageDimension<ImageType>::ImageDimension> MaskImageType;
  typedef typename    MaskImageType::Pointer MaskImagePointer;

  /** Strategy factory typedefs */
  typedef ThresholdStrategy StrategyKeyType;
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

  /** Get/set OutsideValue parameter */
  virtual ImagePixelType GetOutsideValue() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->GetOutsideValue();
      case ThresholdStrategySingle:
        return ptrStrategyFactory->template GetConcreteStrategy<SingleThresholdType>
          (ThresholdStrategySingle)->GetOutsideValue();
      case ThresholdStrategyDouble:
        return ptrStrategyFactory->template GetConcreteStrategy<DoubleThresholdType>
          (ThresholdStrategyDouble)->GetOutsideValue();
      case ThresholdStrategyOtsu:
        return ptrStrategyFactory->template GetConcreteStrategy<OtsuThresholdType>
          (ThresholdStrategyOtsu)->GetOutsideValue();
      case ThresholdStrategyKappaSigma:
        return ptrStrategyFactory->template GetConcreteStrategy<KappaSigmaThresholdType>
          (ThresholdStrategyKappaSigma)->GetOutsideValue();
      default:
        itkExceptionMacro("Current strategy does not support OutsideValue");
      }
    }
  virtual void SetOutsideValue(ImagePixelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->SetOutsideValue(value);
        break;
      case ThresholdStrategySingle:
        ptrStrategyFactory->template GetConcreteStrategy<SingleThresholdType>
          (ThresholdStrategySingle)->SetOutsideValue(value);
        break;
      case ThresholdStrategyDouble:
        ptrStrategyFactory->template GetConcreteStrategy<DoubleThresholdType>
          (ThresholdStrategyDouble)->SetOutsideValue(value);
        break;
      case ThresholdStrategyOtsu:
        ptrStrategyFactory->template GetConcreteStrategy<OtsuThresholdType>
          (ThresholdStrategyOtsu)->SetOutsideValue(value);
        break;
      case ThresholdStrategyKappaSigma:
        ptrStrategyFactory->template GetConcreteStrategy<KappaSigmaThresholdType>
          (ThresholdStrategyKappaSigma)->SetOutsideValue(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support OutsideValue");
      }
    }

  /** Get/set InsideValue parameter */
  virtual ImagePixelType GetInsideValue() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->GetInsideValue();
      case ThresholdStrategyDouble:
        return ptrStrategyFactory->template GetConcreteStrategy<DoubleThresholdType>
          (ThresholdStrategyDouble)->GetInsideValue();
      case ThresholdStrategyOtsu:
        return ptrStrategyFactory->template GetConcreteStrategy<OtsuThresholdType>
          (ThresholdStrategyOtsu)->GetInsideValue();
      case ThresholdStrategyKappaSigma:
        return ptrStrategyFactory->template GetConcreteStrategy<KappaSigmaThresholdType>
          (ThresholdStrategyKappaSigma)->GetInsideValue();
      default:
        itkExceptionMacro("Current strategy does not support InsideValue");
      }
    }
  virtual void SetInsideValue(ImagePixelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->SetInsideValue(value);
        break;
      case ThresholdStrategyDouble:
        ptrStrategyFactory->template GetConcreteStrategy<DoubleThresholdType>
          (ThresholdStrategyDouble)->SetInsideValue(value);
        break;
      case ThresholdStrategyOtsu:
        ptrStrategyFactory->template GetConcreteStrategy<OtsuThresholdType>
          (ThresholdStrategyOtsu)->SetInsideValue(value);
        break;
      case ThresholdStrategyKappaSigma:
        ptrStrategyFactory->template GetConcreteStrategy<KappaSigmaThresholdType>
          (ThresholdStrategyKappaSigma)->SetInsideValue(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support InsideValue");
      }
    }

  /** Get/set LowerThreshold parameter */
  virtual ImagePixelType GetLowerThreshold() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->GetLowerThreshold();
      case ThresholdStrategySingle:
        return ptrStrategyFactory->template GetConcreteStrategy<SingleThresholdType>
          (ThresholdStrategySingle)->GetLower();
      default:
        itkExceptionMacro("Current strategy does not support LowerThreshold");
      }
    }
  virtual void SetLowerThreshold(ImagePixelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->SetLowerThreshold(value);
        break;
      case ThresholdStrategySingle:
        ptrStrategyFactory->template GetConcreteStrategy<SingleThresholdType>
          (ThresholdStrategySingle)->SetLower(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support LowerThreshold");
      }
    }

  /** Get/set UpperThreshold parameter */
  virtual ImagePixelType GetUpperThreshold() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->GetUpperThreshold();
      case ThresholdStrategySingle:
        return ptrStrategyFactory->template GetConcreteStrategy<SingleThresholdType>
          (ThresholdStrategySingle)->GetUpper();
      default:
        itkExceptionMacro("Current strategy does not support UpperThreshold");
      }
    }
  virtual void SetUpperThreshold(ImagePixelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ThresholdStrategyBinary:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdType>
          (ThresholdStrategyBinary)->SetUpperThreshold(value);
        break;
      case ThresholdStrategySingle:
        ptrStrategyFactory->template GetConcreteStrategy<SingleThresholdType>
          (ThresholdStrategySingle)->SetUpper(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support UpperThreshold");
      }
    }

  /** Get/set double threshold filter parameters */
  itkSetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold1, ImagePixelType
  );
  itkGetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold1, ImagePixelType
  );
  itkSetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold2, ImagePixelType
  );
  itkGetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold2, ImagePixelType
  );
  itkSetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold3, ImagePixelType
  );
  itkGetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold3, ImagePixelType
  );
  itkSetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold4, ImagePixelType
  );
  itkGetConcreteStrategyMacro(
    Threshold, DoubleThresholdType,
    Double, Threshold4, ImagePixelType
  );

  /** Get/set Otsu threshold filter parameters */
  itkGetConcreteStrategyMacro(
    Threshold, OtsuThresholdType,
    Otsu, Threshold, ImagePixelType
  );
  itkSetConcreteStrategyMacro(
    Threshold, OtsuThresholdType,
    Otsu, NumberOfHistogramBins, unsigned long
  );
  itkGetConcreteStrategyMacro(
    Threshold, OtsuThresholdType,
    Otsu, NumberOfHistogramBins, unsigned long
  );

  /** Get/set KappaSigma threshold filter parameters */
  itkSetConcreteStrategyMacro(
    Threshold, KappaSigmaThresholdType,
    KappaSigma, MaskValue, MaskImagePixelType
  );
  itkGetConcreteStrategyMacro(
    Threshold, KappaSigmaThresholdType,
    KappaSigma, MaskValue, MaskImagePixelType
  );
  itkSetConcreteStrategyMacro(
    Threshold, KappaSigmaThresholdType,
    KappaSigma, SigmaFactor, double
  );
  itkGetConcreteStrategyMacro(
    Threshold, KappaSigmaThresholdType,
    KappaSigma, SigmaFactor, double
  );
  itkSetConcreteStrategyMacro(
    Threshold, KappaSigmaThresholdType,
    KappaSigma, NumberOfIterations, unsigned int
  );
  itkGetConcreteStrategyMacro(
    Threshold, KappaSigmaThresholdType,
    KappaSigma, NumberOfIterations, unsigned int
  );

protected:
  SimpleThresholdImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1);
    m_StrategyFactory.template AddStrategy<BinaryThresholdType>
      ( ThresholdStrategyBinary );
    m_StrategyFactory.template AddStrategy<SingleThresholdType>
      ( ThresholdStrategySingle );
    m_StrategyFactory.template AddStrategy<DoubleThresholdType>
      ( ThresholdStrategyDouble );
    m_StrategyFactory.template AddStrategy<OtsuThresholdType>
      ( ThresholdStrategyOtsu );
    m_StrategyFactory.template AddStrategy<KappaSigmaThresholdType>
      ( ThresholdStrategyKappaSigma );
  }
  virtual ~SimpleThresholdImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleThresholdImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef BinaryThresholdImageFilter<ImageType,ImageType>
    BinaryThresholdType;
  typedef ThresholdImageFilter<ImageType>
    SingleThresholdType;
  typedef DoubleThresholdImageFilter<ImageType,ImageType>
    DoubleThresholdType;
  typedef OtsuThresholdImageFilter<ImageType,ImageType>
    OtsuThresholdType;
  typedef KappaSigmaThresholdImageFilter<ImageType,MaskImageType,ImageType>
    KappaSigmaThresholdType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
