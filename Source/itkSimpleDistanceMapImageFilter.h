/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleDistanceMapImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleDistanceMapImageFilter_h
#define __itkSimpleDistanceMapImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkStrategyFactory.h"

//#include "itkApproximateSignedDistanceMapImageFilter.h"
//#include "itkFastChamferDistanceImageFilter.h"
#include "itkDanielssonDistanceMapImageFilter.h"
#include "itkSignedDanielssonDistanceMapImageFilter.h"
#include "itkSignedMaurerDistanceMapImageFilter.h"

namespace itk
{

/** List of strategies for distance map filters. */
enum DistanceMapStrategy {
  //DistanceMapStrategyApproximateSigned,
  //DistanceMapStrategyFastChamfer,
  DistanceMapStrategyDanielsson,
  DistanceMapStrategySignedDanielsson,
  DistanceMapStrategySignedMaurer
};

/** \class SimpleDistanceMapImageFilter
 * \brief Implements various distance map filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   TOutputImage: Optional output image type
 *
 * This filter expects the following inputs:
 *   Input1: The binary image for distance map computation
 *
 * This filter produces the following outputs:
 *   Output1: The resultant single- or double-sided distance map
 *
 * This filter currently supports the following strategies:
 *   Danielsson: Computes single-side distance map using method [1]
 *   SignedDanielsson: Computes double-sided distance map using method [1]
 *   SignedMaurer: Computes double-sided distance map using method [2]
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (bool) SquaredDistance Danielsson/SignedDanielsson/SignedMaurer
 *     Specifies whether the distance is squared or not.
 *     Squared distances are computationally less expensive.
 *   (bool) InsideIsPositive SignedDanielsson/SignedMaurer
 *     Specifies whether the distance is positive/negative inside binary object.
 *     True means positive inside and negative outside,
 *     false means negative inside and positive outside.
 *   (bool) UseImageSpacing Danielsson/SignedDanielsson/SignedMaurer
 *     Specifies whether image spacing is taken into account when computing distances.
 *
 * References:
 * [1] Danielsson. Euclidean Distance Mapping. Computer
 *     Graphics and Image Processing 14, 227-248 (1980).
 * [2] Maurer, Qi, and Raghavan, "A Linear Time Algorithm for Computing Exact
 *     Euclidean Distance Transforms of Binary Images in Arbitrary Dimensions",
 *     IEEE Transactions on Pattern Analysis and Machine Intelligence,
 *     25(2): 265-270, 2003.
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage, class TOutputImage=TInputImage>
class ITK_EXPORT SimpleDistanceMapImageFilter :
  public ImageToImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleDistanceMapImageFilter                  Self;
  typedef ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleDistanceMapImageFilter, ImageToImageFilter);

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
  typedef DistanceMapStrategy StrategyKeyType;
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

  /** Get/set SquaredDistance parameter */
  virtual bool GetSquaredDistance() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case DistanceMapStrategyDanielsson:
        return ptrStrategyFactory->template GetConcreteStrategy<DanielssonType>
          (DistanceMapStrategyDanielsson)->GetSquaredDistance();
      case DistanceMapStrategySignedDanielsson:
        return ptrStrategyFactory->template GetConcreteStrategy<SignedDanielssonType>
          (DistanceMapStrategySignedDanielsson)->GetSquaredDistance();
      case DistanceMapStrategySignedMaurer:
        return ptrStrategyFactory->template GetConcreteStrategy<SignedMaurerType>
          (DistanceMapStrategySignedMaurer)->GetSquaredDistance();
      default:
        itkExceptionMacro("Current strategy does not support SquaredDistance");
      }
    }
  virtual void SetSquaredDistance(bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case DistanceMapStrategyDanielsson:
        ptrStrategyFactory->template GetConcreteStrategy<DanielssonType>
          (DistanceMapStrategyDanielsson)->SetSquaredDistance(value);
        break;
      case DistanceMapStrategySignedDanielsson:
        ptrStrategyFactory->template GetConcreteStrategy<SignedDanielssonType>
          (DistanceMapStrategySignedDanielsson)->SetSquaredDistance(value);
        break;
      case DistanceMapStrategySignedMaurer:
        ptrStrategyFactory->template GetConcreteStrategy<SignedMaurerType>
          (DistanceMapStrategySignedMaurer)->SetSquaredDistance(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support SquaredDistance");
      }
    }

  /** Get/set InsideIsPositive parameter */
  virtual bool GetInsideIsPositive() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case DistanceMapStrategySignedDanielsson:
        ptrStrategyFactory->template GetConcreteStrategy<SignedDanielssonType>
          (DistanceMapStrategySignedDanielsson)->GetInsideIsPositive();
      case DistanceMapStrategySignedMaurer:
        ptrStrategyFactory->template GetConcreteStrategy<SignedMaurerType>
          (DistanceMapStrategySignedMaurer)->GetInsideIsPositive();
      default:
        itkExceptionMacro("Current strategy does not support InsideIsPositive");
      }
    }
  virtual void SetInsideIsPositive(bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case DistanceMapStrategySignedDanielsson:
        ptrStrategyFactory->template GetConcreteStrategy<SignedDanielssonType>
          (DistanceMapStrategySignedDanielsson)->SetInsideIsPositive(value);
        break;
      case DistanceMapStrategySignedMaurer:
        ptrStrategyFactory->template GetConcreteStrategy<SignedMaurerType>
          (DistanceMapStrategySignedMaurer)->SetInsideIsPositive(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support InsideIsPositive");
      }
    }

  /** Get/set UseImageSpacing parameter */
  virtual bool GetUseImageSpacing() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case DistanceMapStrategyDanielsson:
        return ptrStrategyFactory->template GetConcreteStrategy<DanielssonType>
          (DistanceMapStrategyDanielsson)->GetUseImageSpacing();
      case DistanceMapStrategySignedDanielsson:
        return ptrStrategyFactory->template GetConcreteStrategy<SignedDanielssonType>
          (DistanceMapStrategySignedDanielsson)->GetUseImageSpacing();
      case DistanceMapStrategySignedMaurer:
        return ptrStrategyFactory->template GetConcreteStrategy<SignedMaurerType>
          (DistanceMapStrategySignedMaurer)->GetUseImageSpacing();
      default:
        itkExceptionMacro("Current strategy does not support InsideIsPositive");
      }
    }
  virtual void SetUseImageSpacing(bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case DistanceMapStrategyDanielsson:
        ptrStrategyFactory->template GetConcreteStrategy<DanielssonType>
          (DistanceMapStrategyDanielsson)->SetUseImageSpacing(value);
        break;
      case DistanceMapStrategySignedDanielsson:
        ptrStrategyFactory->template GetConcreteStrategy<SignedDanielssonType>
          (DistanceMapStrategySignedDanielsson)->SetUseImageSpacing(value);
        break;
      case DistanceMapStrategySignedMaurer:
        ptrStrategyFactory->template GetConcreteStrategy<SignedMaurerType>
          (DistanceMapStrategySignedMaurer)->SetUseImageSpacing(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support InsideIsPositive");
      }
    }

protected:
  SimpleDistanceMapImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1);
    //m_StrategyFactory.template AddStrategy<ApproximateSignedDistanceMapType>
    //  ( DistanceMapStrategyApproximateSigned );
    //m_StrategyFactory.template AddStrategy<FastChamferDistanceMapType>
    //  ( DistanceMapStrategyFastChamfer );
    m_StrategyFactory.template AddStrategy<DanielssonType>
      ( DistanceMapStrategyDanielsson );
    m_StrategyFactory.template AddStrategy<SignedDanielssonType>
      ( DistanceMapStrategySignedDanielsson );
    m_StrategyFactory.template AddStrategy<SignedMaurerType>
      ( DistanceMapStrategySignedMaurer );
  }
  virtual ~SimpleDistanceMapImageFilter() {};

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
  SimpleDistanceMapImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  //typedef ApproximateSignedDistanceMapImageFilter<InputImageType,OutputImageType>
  //  ApproximateSignedType;
  //typedef FastChamferDistanceImageFilter<InputImageType,OutputImageType>
  //  FastChamferType;
  typedef DanielssonDistanceMapImageFilter<InputImageType,OutputImageType>
    DanielssonType;
  typedef SignedDanielssonDistanceMapImageFilter<InputImageType,OutputImageType>
    SignedDanielssonType;
  typedef SignedMaurerDistanceMapImageFilter<InputImageType,OutputImageType>
    SignedMaurerType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
