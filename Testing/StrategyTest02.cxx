/*=========================================================================

Program:   Insight Segmentation & Registration Toolkit
Module:    $RCSfile: StrategyTest02.cxx,v $
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
#include "itkSimpleGradientImageFilter.h"
#include "itkSimpleGradientMagnitudeImageFilter.h"
#include "itkSimpleThresholdImageFilter.h"

// Gradient ======================================
template <typename TPixel, unsigned int VDimension>
int TestGradientTemplate(int argc, char * argv [])
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
    itk::GradientStrategy Strategy =
      static_cast<itk::GradientStrategy>( atoi(argv[arg++]) );
    double Sigma = (argc > arg) ? atof( argv[arg++] ) : 1.0;

    std::cout << "Test=" << "GradientMagnitude" << std::endl;
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
    typedef itk::SimpleGradientImageFilter<ImageType> GradientType;
    typedef itk::ImageFileWriter<typename GradientType::OutputImageType> WriterType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename GradientType::Pointer filter = GradientType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetRecursiveGaussianSigma(Sigma);
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
int TestGradientUC2(int argc, char * argv [])
{
  return TestGradientTemplate<unsigned char, 2>(argc, argv);
}
int TestGradientUC3(int argc, char * argv [])
{
  return TestGradientTemplate<unsigned char, 3>(argc, argv);
}
int TestGradientF2(int argc, char * argv [])
{
  return TestGradientTemplate<float, 2>(argc, argv);
}
int TestGradientF3(int argc, char * argv [])
{
  return TestGradientTemplate<float, 3>(argc, argv);
}

// GradientMagnitude ======================================
template <typename TPixel, unsigned int VDimension>
int TestGradientMagnitudeTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 3 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "[Sigma] [KernelRadius] ";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];
    itk::GradientMagnitudeStrategy Strategy =
      static_cast<itk::GradientMagnitudeStrategy>( atoi(argv[arg++]) );
    double Sigma = (argc > arg) ? atof( argv[arg++] ) : 1.0;
    unsigned int KernelRadius = (argc > arg) ? atoi( argv[arg++] ) : 0;

    std::cout << "Test=" << "GradientMagnitude" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "Sigma=" << Sigma << std::endl;
    std::cout << "KernelRadius=" << (double)KernelRadius << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::ImageFileWriter<ImageType> WriterType;
    typedef itk::SimpleGradientMagnitudeImageFilter<ImageType,ImageType> GradientMagnitudeType;
    typedef typename GradientMagnitudeType::KernelType KernelType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename GradientMagnitudeType::Pointer filter = GradientMagnitudeType::New();
    if (Strategy != itk::GradientMagnitudeStrategyLaplacianRecursiveGaussian)
      {
      // NOTE: LaplacianRecursiveGaussian strategy does not have Filter progress
      itk::SimpleFilterWatcher watcher(filter);
      }
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetRecursiveGaussianSigma(Sigma);
    if (KernelRadius > 0)
      {
      typename KernelType::SizeType kernelSize;
      kernelSize.Fill(KernelRadius);
      KernelType kernel = KernelType::Ball(kernelSize);
      filter->SetMorphologicalKernel(kernel);
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
int TestGradientMagnitudeUC2(int argc, char * argv [])
{
  return TestGradientMagnitudeTemplate<unsigned char, 2>(argc, argv);
}
int TestGradientMagnitudeUC3(int argc, char * argv [])
{
  return TestGradientMagnitudeTemplate<unsigned char, 3>(argc, argv);
}
int TestGradientMagnitudeF2(int argc, char * argv [])
{
  return TestGradientMagnitudeTemplate<float, 2>(argc, argv);
}
int TestGradientMagnitudeF3(int argc, char * argv [])
{
  return TestGradientMagnitudeTemplate<float, 3>(argc, argv);
}

// Threshold ==============================================
template <typename TPixel, unsigned int VDimension>
int TestThresholdTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 3 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "[Outside] [Inside] [Lower] [Upper]";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];

    // Read strategy from command line
    itk::ThresholdStrategy Strategy =
      static_cast<itk::ThresholdStrategy>( atoi(argv[arg++]) );

    // Read other parameters
    TPixel Outside = (argc > arg) ? (TPixel)atof(argv[arg++]) : (TPixel)0;
    TPixel Inside = (argc > arg) ? (TPixel)atof(argv[arg++]) : (TPixel)0;
    TPixel Lower = (argc > arg) ? (TPixel)atof(argv[arg++]) : (TPixel)0;
    TPixel Upper = (argc > arg) ? (TPixel)atof(argv[arg++]) : (TPixel)0;

    std::cout << "Test=" << "Threshold" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "Outside=" << (double)Outside << std::endl;
    std::cout << "Inside=" << (double)Inside << std::endl;
    std::cout << "Lower=" << (double)Lower << std::endl;
    std::cout << "Upper=" << (double)Upper << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::ImageFileWriter<ImageType> WriterType;
    typedef itk::SimpleThresholdImageFilter<ImageType> ThresholdType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename ThresholdType::Pointer filter = ThresholdType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetOutsideValue(Outside);
    if (Strategy != itk::ThresholdStrategySingle)
      {
      filter->SetInsideValue(Inside);
      }
    if ((Strategy == itk::ThresholdStrategySingle) ||
        (Strategy == itk::ThresholdStrategyBinary))
      {
      filter->SetLowerThreshold(Lower);
      filter->SetUpperThreshold(Upper);
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
int TestThresholdUC2(int argc, char * argv [])
{
  return TestThresholdTemplate<unsigned char, 2>(argc, argv);
}
int TestThresholdUC3(int argc, char * argv [])
{
  return TestThresholdTemplate<unsigned char, 3>(argc, argv);
}
int TestThresholdF2(int argc, char * argv [])
{
  return TestThresholdTemplate<float, 2>(argc, argv);
}
int TestThresholdF3(int argc, char * argv [])
{
  return TestThresholdTemplate<float, 3>(argc, argv);
}