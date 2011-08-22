/*=========================================================================

Program:   Insight Segmentation & Registration Toolkit
Module:    $RCSfile: StrategyTest03.cxx,v $
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
#include "itkSimpleFilterWatcher.h"
#include "itkSimpleProjectionImageFilter.h"
#include "itkSimpleMorphologyImageFilter.h"
#include "itkSimpleSmoothImageFilter.h"

// Projection =============================================
template <typename TPixel, unsigned int VInputDimension, unsigned int VOutputDimension>
int TestProjectionTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 4 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "ProjectionDimension";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];
    itk::ProjectionStrategy Strategy =
      static_cast<itk::ProjectionStrategy>( atoi(argv[arg++]) );
    unsigned int ProjectionDimension = atoi( argv[arg++] );

    std::cout << "Test=" << "Projection" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "InputDimension=" << VInputDimension << std::endl;
    std::cout << "OutputDimension=" << VOutputDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "ProjectionDimension=" << ProjectionDimension << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int InputDimension = VInputDimension;
    static const unsigned int OutputDimension = VOutputDimension;
    typedef itk::Image<PixelType, InputDimension> InputImageType;
    typedef itk::Image<PixelType, OutputDimension> OutputImageType;
    typedef itk::ImageFileReader<InputImageType> ReaderType;
    typedef itk::SimpleProjectionImageFilter<InputImageType,OutputImageType> ProjectionType;
    typedef itk::ImageFileWriter<OutputImageType> WriterType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename InputImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename ProjectionType::Pointer filter = ProjectionType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetProjectionDimension(ProjectionDimension);
    filter->Update();
    typename OutputImageType::Pointer output = filter->GetOutput();
    // NOTE: Disconnection is required because there is an issue with projection filters.
    // If the output is not disconnected, when the below writer updates, the filter sets
    // the filter output (writer input) requested region back to the filter input buffered
    // region, causing the writer to throw an exception.
    output->DisconnectPipeline();

    // Write output
    typename WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(OutputFilename);
    writer->SetInput(output);
    writer->Update();

    return EXIT_SUCCESS;
    }
  catch (itk::ExceptionObject & err)
    {
    std::cerr << "ExceptionObject caught !" << std::endl; 
    std::cerr << err << std::endl; 
    return EXIT_FAILURE;
    }
}
int TestProjectionUC3UC2(int argc, char * argv [])
{
  return TestProjectionTemplate<unsigned char, 3, 2>(argc, argv);
}
int TestProjectionF3F2(int argc, char * argv [])
{
  return TestProjectionTemplate<float, 3, 2>(argc, argv);
}


// Morphology =============================================
template <typename TPixel, unsigned int VDimension>
int TestMorphologyTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 3 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "[KernelLength]";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];
    itk::MorphologyStrategy Strategy =
      static_cast<itk::MorphologyStrategy>( atoi(argv[arg++]) );
    unsigned int KernelLength = (argc > arg) ? atoi( argv[arg++] ) : 0;

    std::cout << "Test=" << "Morphology" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "KernelLength=" << KernelLength << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::SimpleMorphologyImageFilter<ImageType,ImageType> MorphologyType;
    typedef typename MorphologyType::KernelType KernelType;
    typedef itk::ImageFileWriter<ImageType> WriterType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename MorphologyType::Pointer filter = MorphologyType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    typename KernelType::SizeType kernelRadius;
    kernelRadius.Fill(KernelLength);
    KernelType kernel = KernelType::Ball(kernelRadius);
    try
      {
      filter->SetKernel(kernel);
      }
    catch (...)
      {
      // Kernel is only supported by some algorithms
      }
    filter->Update();

    // Write output
    typename WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(OutputFilename);
    writer->SetInput(filter->GetOutput());
    writer->Update();

    return EXIT_SUCCESS;
    }
  catch (itk::ExceptionObject & err)
    {
    std::cerr << "ExceptionObject caught !" << std::endl; 
    std::cerr << err << std::endl; 
    return EXIT_FAILURE;
    }
}
int TestMorphologyUC2(int argc, char * argv [])
{
  return TestMorphologyTemplate<unsigned char, 2>(argc, argv);
}
int TestMorphologyUC3(int argc, char * argv [])
{
  return TestMorphologyTemplate<unsigned char, 3>(argc, argv);
}
int TestMorphologyF2(int argc, char * argv [])
{
  return TestMorphologyTemplate<float, 2>(argc, argv);
}
int TestMorphologyF3(int argc, char * argv [])
{
  return TestMorphologyTemplate<float, 3>(argc, argv);
}


// Smooth =================================================
template <typename TPixel, unsigned int VDimension>
int TestSmoothTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 3 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "[Sigma]";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];
    itk::SmoothStrategy Strategy =
      static_cast<itk::SmoothStrategy>( atoi(argv[arg++]) );
    double Sigma = (argc > arg) ? atoi( argv[arg++] ) : 1.0;

    std::cout << "Test=" << "Smooth" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "Sigma=" << Sigma << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::SimpleSmoothImageFilter<ImageType,ImageType> SmoothType;
    typedef itk::ImageFileWriter<ImageType> WriterType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename SmoothType::Pointer filter = SmoothType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetBilateralDomainSigma(Sigma);
    filter->SetBilateralRangeSigma(Sigma/5.0);
    filter->SetCurvatureFlowTimeStep(0.1);
    filter->SetCurvatureFlowNumberOfIterations(10);
    typename SmoothType::DiscreteGaussianType::ArrayType variance;
    variance.Fill(Sigma);
    filter->SetDiscreteGaussianVariance(variance);
    filter->SetRecursiveGaussianSigma(Sigma);
    typename SmoothType::MedianType::InputSizeType radiusSize;
    radiusSize.Fill(Sigma);
    filter->SetMedianRadius(radiusSize);
    filter->Update();

    // Write output
    typename WriterType::Pointer writer = WriterType::New();
    writer->SetFileName(OutputFilename);
    writer->SetInput(filter->GetOutput());
    writer->Update();

    return EXIT_SUCCESS;
    }
  catch (itk::ExceptionObject & err)
    {
    std::cerr << "ExceptionObject caught !" << std::endl; 
    std::cerr << err << std::endl; 
    return EXIT_FAILURE;
    }
}
int TestSmoothUC2(int argc, char * argv [])
{
  return TestSmoothTemplate<unsigned char, 2>(argc, argv);
}
int TestSmoothUC3(int argc, char * argv [])
{
  return TestSmoothTemplate<unsigned char, 3>(argc, argv);
}
int TestSmoothF2(int argc, char * argv [])
{
  return TestSmoothTemplate<float, 2>(argc, argv);
}
int TestSmoothF3(int argc, char * argv [])
{
  return TestSmoothTemplate<float, 3>(argc, argv);
}