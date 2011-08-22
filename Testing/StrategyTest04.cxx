/*=========================================================================

Program:   Insight Segmentation & Registration Toolkit
Module:    $RCSfile: StrategyTest04.cxx,v $
Language:  C++
Date:      $Date$
Version:   $Revision$

Copyright (c) Insight Software Consortium. All rights reserved.
See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

   This software is distributed WITHOUT ANY WARRANTY; without even 
   the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
   PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#if defined(_MSC_VER)
#pragma warning ( disable : 4786 )
#endif

#define _SCL_SECURE_NO_WARNINGS

#include <iostream>
#include <sstream>

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkSimpleTransform.h"
#include "itkSimpleOptimizer.h"
#include "itkSimpleImageToImageMetric.h"
#include "itkSimpleInterpolateImageFunction.h"
#include "itkImageRegistrationMethod.h"
#include "itkMersenneTwisterRandomVariateGenerator.h"
#include "itkResampleImageFilter.h"

// CommandIterationUpdate =================================
class CommandIterationUpdate : public itk::Command
{
public:
  typedef  CommandIterationUpdate Self;
  typedef  itk::Command           Superclass;
  typedef itk::SmartPointer<Self> Pointer;
  itkNewMacro( Self );
protected:
  CommandIterationUpdate() {};
public:
  typedef itk::SimpleOptimizer OptimizerType;
  typedef const OptimizerType* OptimizerPointer;
  void Execute(itk::Object *caller, const itk::EventObject & eventObj)
  {
    Execute((const itk::Object *)caller, eventObj);
  }
  void Execute(const itk::Object * object, const itk::EventObject & eventObj)
  {
    OptimizerPointer optimizer = dynamic_cast<OptimizerPointer>( object );
    if(!itk::IterationEvent().CheckEvent( &eventObj ))
      {
      return;
      }
    std::cout << "Iteration=" << optimizer->GetCurrentPosition() << std::endl;
  }
};

// Registration =============================================
template <typename TPixel, typename TCoordRep, unsigned int VDimension>
int TestRegistrationTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 8 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "FixedFilename MovingFilename OutputFilename ";
      std::cout << "TransformStrategy Optimizer Metric Interpolator ";
      std::cout << "InitialParameter1 InitialParameter2 ...";
      std::cout << "Scales1 Scales2 ...";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* FixedFilename = argv[arg++];
    char* MovingFilename = argv[arg++];
    char* OutputFilename = argv[arg++];

    // Read strategies from command line
    itk::TransformStrategy TransformStrategy =
      static_cast<itk::TransformStrategy>( atoi(argv[arg++]) );
    itk::OptimizerStrategy OptimizerStrategy =
      static_cast<itk::OptimizerStrategy>( atoi(argv[arg++]) );
    itk::ImageToImageMetricStrategy MetricStrategy =
      static_cast<itk::ImageToImageMetricStrategy>( atoi(argv[arg++]) );
    itk::InterpolateImageFunctionStrategy InterpolatorStrategy =
      static_cast<itk::InterpolateImageFunctionStrategy>( atoi(argv[arg++]) );

    std::cout << "Test=" << "Registration" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "CoordRepType=" << typeid(TCoordRep).name() << std::endl;
    std::cout << "FixedFilename=" << FixedFilename << std::endl;
    std::cout << "MovingFilename=" << MovingFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "TransformStrategy=" << TransformStrategy << std::endl;
    std::cout << "Optimizer=" << OptimizerStrategy << std::endl;
    std::cout << "Metric=" << MetricStrategy << std::endl;
    std::cout << "Interpolator=" << InterpolatorStrategy << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    typedef TCoordRep CoordRepType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::ImageFileWriter<ImageType> WriterType;
    typedef itk::SimpleTransform<CoordRepType,Dimension,Dimension> TransformType;
    typedef itk::SimpleOptimizer OptimizerType;
    typedef itk::SimpleImageToImageMetric<ImageType,ImageType> MetricType;
    typedef itk::SimpleInterpolateImageFunction<ImageType,CoordRepType> InterpolatorType;
    typedef itk::ImageRegistrationMethod<ImageType,ImageType> RegistrationType;
    typedef itk::ResampleImageFilter<ImageType,ImageType,CoordRepType> ResampleType;

    // Read fixed image
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(FixedFilename);
    reader->Update();
    typename ImageType::Pointer fixedImage = reader->GetOutput();
    fixedImage->DisconnectPipeline();

    // Transform
    typename TransformType::Pointer transform = TransformType::New();
    transform->SetStrategy(TransformStrategy);
    typename TransformType::ParametersType initial(transform->GetNumberOfParameters());
    if (initial.Size() > (argc-arg))
      {
        std::cerr << "Incorrect number of InitialParameters" << std::endl;
        return EXIT_FAILURE;
      }
    for (unsigned int i=0; i<initial.Size(); i++)
      {
      initial[i] = (typename TransformType::ParametersValueType)atof(argv[arg+i]);
      }
    arg += initial.Size();
    transform->SetParameters(initial);
    std::cout << "InitialParameters=" << initial << std::endl;

    // Set fixed parameters
    typename TransformType::ParametersType fixedParams = transform->GetFixedParameters();
    if (fixedParams.Size() == Dimension) // If fixed size == dim, use image center
      {
      // Compute center of image
      typename ImageType::IndexType index;
      for (unsigned int i=0; i<Dimension; i++)
        {
        index[i] = fixedImage->GetLargestPossibleRegion().GetSize()[i] / 2;
        }
      typename ImageType::PointType center;
      fixedImage->TransformIndexToPhysicalPoint(index, center);
      // Set fixed parameters
      for (unsigned int i=0; i<Dimension; i++)
        {
        fixedParams[i] = center[i];
        }
      transform->SetFixedParameters(fixedParams);
      std::cout << "FixedParameters=" << fixedParams << std::endl;
      }

    // Interpolator
    typename InterpolatorType::Pointer interpolator = InterpolatorType::New();
    interpolator->SetStrategy(InterpolatorStrategy);

    // Prepare moving image
    typename ResampleType::Pointer resample = ResampleType::New();
    resample->SetInput(fixedImage);
    resample->SetTransform(transform);
    resample->SetInterpolator(interpolator);
    resample->SetOutputParametersFromImage(fixedImage);
    resample->Update();
    typename ImageType::Pointer movingImage = resample->GetOutput();
    movingImage->DisconnectPipeline();
    typename WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(MovingFilename);
    writer->SetInput(movingImage);
    writer->Update();

    // Optimizer
    typename OptimizerType::Pointer optimizer = OptimizerType::New();
    optimizer->SetStrategy(OptimizerStrategy);
    typename CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
    optimizer->AddObserver(itk::AnyEvent(), observer);
    typename OptimizerType::ScalesType scales(transform->GetNumberOfParameters());
    if (scales.Size() > (argc-arg))
      {
      scales.Fill(1.0);
      }
    else
      {
      for (unsigned int i=0; i<scales.Size(); i++)
        {
        scales[i] = (typename OptimizerType::ScalesType::ValueType)atof(argv[arg+i]);
        }
      }
    optimizer->SetScales(scales);
    std::cout << "Scales=" << scales << std::endl;

    // Metric
    typename MetricType::Pointer metric = MetricType::New();
    metric->SetStrategy(MetricStrategy);
    metric->SetFixedImageRegion(fixedImage->GetLargestPossibleRegion());

    // Reset transform
    initial.Fill(0.0);
    transform->SetParameters(initial);
    try
      {
      transform->SetIdentity();
      }
    catch (...)
      {
        if (
          TransformStrategy != itk::TransformStrategyTranslation &&
          TransformStrategy != itk::TransformStrategyScale &&
          TransformStrategy != itk::TransformStrategyBSpline
        )
          {
            std::cerr << "SetIdentity failed for supported transform" << std::endl;
            return EXIT_FAILURE;
          }
      }

    // Registration
    typename RegistrationType::Pointer registration = RegistrationType::New();
    registration->SetFixedImage(fixedImage);
    registration->SetMovingImage(movingImage);
    registration->SetOptimizer(optimizer);
    registration->SetMetric(metric);
    registration->SetTransform(transform);
    registration->SetInterpolator(interpolator);
    registration->SetInitialTransformParameters(initial);
    registration->Initialize();
    registration->StartRegistration();

    // Write final parameters
    typename TransformType::ParametersType final = transform->GetParameters();
    std::cout << "Final parameters=" << final << std::endl;

    // Write output
    resample->SetInput(movingImage);
    resample->SetTransform(transform);
    resample->SetInterpolator(interpolator);
    resample->SetOutputParametersFromImage(fixedImage);
    resample->Update();
    typename ImageType::Pointer outputImage = resample->GetOutput();
    outputImage->DisconnectPipeline();
    writer->SetFileName(OutputFilename);
    writer->SetInput(outputImage);
    writer->Update();

    // Ensure input parameters match found parameters (within tolerance)
    const CoordRepType Tolerance = 0.1;
    for (unsigned int i=0; i<final.Size(); i++)
      {
      if (abs((final[i] - initial[i]) > Tolerance))
        {
        std::cerr << "Final parameters do not match initial parameters" << std::endl;
        return EXIT_FAILURE;
        }
      }

    return EXIT_SUCCESS;
    }
  catch (itk::ExceptionObject & err)
    {
    std::cerr << "ExceptionObject caught !" << std::endl; 
    std::cerr << err << std::endl; 
    return EXIT_FAILURE;
    }
}
int TestRegistrationUC2(int argc, char * argv [])
{
  return TestRegistrationTemplate<unsigned char, double, 2>(argc, argv);
}
int TestRegistrationUC3(int argc, char * argv [])
{
  return TestRegistrationTemplate<unsigned char, double, 3>(argc, argv);
}
int TestRegistrationF2(int argc, char * argv [])
{
  return TestRegistrationTemplate<float, double, 2>(argc, argv);
}
int TestRegistrationF3(int argc, char * argv [])
{
  return TestRegistrationTemplate<float, double, 3>(argc, argv);
}