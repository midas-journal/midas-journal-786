/*=========================================================================

 Program:   Insight Segmentation & Registration Toolkit
 Module:    itkSimpleProjectionImageFilter.h
 Language:  C++
 Date:      $Date$
 Version:   $Revision$

 Copyright (c) Insight Software Consortium. All rights reserved.
 See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

    This software is distributed WITHOUT ANY WARRANTY; without even
    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
    PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleProjectionImageFilter_h
#define __itkSimpleProjectionImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkProjectionImageFilter.h"
#include "itkStrategyFactory.h"

#include "itkBinaryProjectionImageFilter.h"
#include "itkBinaryThresholdProjectionImageFilter.h"
#include "itkMaximumProjectionImageFilter.h"
#include "itkMinimumProjectionImageFilter.h"
#include "itkSumProjectionImageFilter.h"
#include "itkMeanProjectionImageFilter.h"
#include "itkStandardDeviationProjectionImageFilter.h"
#include "itkMedianProjectionImageFilter.h"

namespace itk
{

/** List of strategies for projection image filtering. */
  enum ProjectionStrategy {
  ProjectionStrategyBinary,
  ProjectionStrategyBinaryThreshold,
  ProjectionStrategyMaximum,
  ProjectionStrategyMinimum,
  ProjectionStrategySum,
  ProjectionStrategyMean,
  ProjectionStrategyStandardDeviation,
  ProjectionStrategyMedian
};

/** Create a fake accumulator for the template parameters.
 *  This is needed so that the ProjectionImageFilter
 *  GenerateOutputInformation and GenerateInputRequestedRegion
 *  methods are inherited. */
namespace Function {
template <class TInputPixel>
class FakeAccumulator
{
public:
  FakeAccumulator( unsigned long ) {}
  ~FakeAccumulator(){}
  inline void Initialize() {}
  inline void operator()( const TInputPixel &input ) { }
  inline TInputPixel GetValue() { return NumericTraits<TInputPixel>::Zero; }
};
} // end namespace Function

/** \class SimpleProjectionImageFilter
 * \brief Implements various projection image filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   ToutputImage: Optional output image type
 *
 * This filter expects the following inputs:
 *   Input1: The image to apply the projection
 *
 * This filter produces the following outputs:
 *   Output1: The projected image, typically Dimension-1
 *
 * This filter currently supports the following strategies (see [1]):
 *   Binary: accumulates the foreground value
 *   BinaryThreshold: accumulates the thresholded value
 *   Maximum: accumulates the maximum value
 *   Minimum: accumulates the minimum value
 *   Sum: accumulates the summation of all values
 *   Mean: accumulates the mean of all values
 *   StandardDeviation: accumulates the standard deviation of all values
 *   Median: accumulates the median of all values
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (unsigned int) ProjectionDimension All
 *     Specifies along which dimension to project or accumulate.
 *
 * References:
 * [1] http://hdl.handle.net/1926/164
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TInputImage,class TOutputImage>
class ITK_EXPORT SimpleProjectionImageFilter :
  public ProjectionImageFilter<TInputImage,TOutputImage,Function::FakeAccumulator<typename TInputImage::PixelType> >
{
public:
  /** Standard class typedefs. */
  typedef SimpleProjectionImageFilter Self;
  typedef ProjectionImageFilter<TInputImage,TOutputImage,Function::FakeAccumulator<typename TInputImage::PixelType> > Superclass;
  typedef SmartPointer<Self> Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);

  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleProjectionImageFilter, ProjectionImageFilter);

  /** Some typedefs. */
  typedef TInputImage                              InputImageType;
  typedef typename    InputImageType::ConstPointer InputImagePointer;
  typedef typename    InputImageType::RegionType   InputRegionType;
  typedef typename    InputImageType::PixelType    InputPixelType;
  typedef TOutputImage                             OutputImageType;
  typedef typename     OutputImageType::Pointer    OutputImagePointer;
  typedef typename     OutputImageType::RegionType OutputRegionType;
  typedef typename     OutputImageType::PixelType  OutputPixelType;

  /** Strategy factory typedefs. */
  typedef ProjectionStrategy StrategyKeyType;
  // NOTE: The StrategyType can not be Superclass because of the templated Accumulator.
  typedef ImageToImageFilter<InputImageType,OutputImageType> StrategyType;
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

  /** Get/set ProjectionDimension parameter */
  virtual const unsigned int& GetProjectionDimension() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case ProjectionStrategyBinary:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryProjectionType>
          (ProjectionStrategyBinary)->GetProjectionDimension();
      case ProjectionStrategyBinaryThreshold:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdProjectionType>(
          ProjectionStrategyBinaryThreshold)->GetProjectionDimension();
      case ProjectionStrategyMaximum:
        return ptrStrategyFactory->template GetConcreteStrategy<MaximumProjectionType>
          (ProjectionStrategyMaximum)->GetProjectionDimension();
      case ProjectionStrategyMinimum:
        return ptrStrategyFactory->template GetConcreteStrategy<MinimumProjectionType>
          (ProjectionStrategyMinimum)->GetProjectionDimension();
      case ProjectionStrategySum:
        return ptrStrategyFactory->template GetConcreteStrategy<SumProjectionType>
          (ProjectionStrategySum)->GetProjectionDimension();
      case ProjectionStrategyMean:
        return ptrStrategyFactory->template GetConcreteStrategy<MeanProjectionType>
          (ProjectionStrategyMean)->GetProjectionDimension();
      case ProjectionStrategyStandardDeviation:
        return ptrStrategyFactory->template GetConcreteStrategy<StandardDeviationProjectionType>
          (ProjectionStrategyStandardDeviation)->GetProjectionDimension();
      case ProjectionStrategyMedian:
        return ptrStrategyFactory->template GetConcreteStrategy<MedianProjectionType>
          (ProjectionStrategyMedian)->GetProjectionDimension();
      default:
        itkExceptionMacro("Current strategy does not support ProjectionDimension");
      }
    }
  virtual void SetProjectionDimension(const unsigned int value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);

    switch (m_StrategyFactory.GetKey())
      {
      case ProjectionStrategyBinary:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryProjectionType>
          (ProjectionStrategyBinary)->SetProjectionDimension(value);
        break;
      case ProjectionStrategyBinaryThreshold:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryThresholdProjectionType>
          (ProjectionStrategyBinaryThreshold)->SetProjectionDimension(value);
        break;
      case ProjectionStrategyMaximum:
        ptrStrategyFactory->template GetConcreteStrategy<MaximumProjectionType>
          (ProjectionStrategyMaximum)->SetProjectionDimension(value);
        break;
      case ProjectionStrategyMinimum:
        ptrStrategyFactory->template GetConcreteStrategy<MinimumProjectionType>
          (ProjectionStrategyMinimum)->SetProjectionDimension(value);
        break;
      case ProjectionStrategySum:
        ptrStrategyFactory->template GetConcreteStrategy<SumProjectionType>
          (ProjectionStrategySum)->SetProjectionDimension(value);
        break;
      case ProjectionStrategyMean:
        ptrStrategyFactory->template GetConcreteStrategy<MeanProjectionType>
          (ProjectionStrategyMean)->SetProjectionDimension(value);
        break;
      case ProjectionStrategyStandardDeviation:
        ptrStrategyFactory->template GetConcreteStrategy<StandardDeviationProjectionType>
          (ProjectionStrategyStandardDeviation)->SetProjectionDimension(value);
        break;
      case ProjectionStrategyMedian:
        ptrStrategyFactory->template GetConcreteStrategy<MedianProjectionType>(
          ProjectionStrategyMedian)->SetProjectionDimension(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support ProjectionDimension");
      }
    }

protected:
  SimpleProjectionImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
    {
    this->SetNumberOfRequiredInputs(1);
    m_StrategyFactory.template AddStrategy<BinaryProjectionType>
      ( ProjectionStrategyBinary );
    m_StrategyFactory.template AddStrategy<BinaryThresholdProjectionType>
      ( ProjectionStrategyBinaryThreshold );
    m_StrategyFactory.template AddStrategy<MaximumProjectionType>
      ( ProjectionStrategyMaximum );
    m_StrategyFactory.template AddStrategy<MinimumProjectionType>
      ( ProjectionStrategyMinimum );
    m_StrategyFactory.template AddStrategy<SumProjectionType>
      ( ProjectionStrategySum );
    m_StrategyFactory.template AddStrategy<MeanProjectionType>
      ( ProjectionStrategyMean );
    m_StrategyFactory.template AddStrategy<StandardDeviationProjectionType>
      ( ProjectionStrategyStandardDeviation );
    m_StrategyFactory.template AddStrategy<MedianProjectionType>
      ( ProjectionStrategyMedian );
    }
  virtual ~SimpleProjectionImageFilter() {};

  void GenerateData()
    {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    //filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
    }

private:
  SimpleProjectionImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef BinaryProjectionImageFilter<InputImageType,OutputImageType>
    BinaryProjectionType;
  typedef BinaryThresholdProjectionImageFilter<InputImageType,OutputImageType>
    BinaryThresholdProjectionType;
  typedef MaximumProjectionImageFilter<InputImageType,OutputImageType>
    MaximumProjectionType;
  typedef MinimumProjectionImageFilter<InputImageType,OutputImageType>
    MinimumProjectionType;
  typedef SumProjectionImageFilter<InputImageType,OutputImageType>
    SumProjectionType;
  typedef MeanProjectionImageFilter<InputImageType,OutputImageType>
    MeanProjectionType;
  typedef StandardDeviationProjectionImageFilter<InputImageType,OutputImageType>
    StandardDeviationProjectionType;
  typedef MedianProjectionImageFilter<InputImageType,OutputImageType>
    MedianProjectionType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
