/*=========================================================================

Program:   Insight Segmentation & Registration Toolkit
Module:    $RCSfile: StrategyTest01.cxx,v $
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

#include "itkTestMain.h"

#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkSimpleFilterWatcher.h"
#include "itkSimpleUnaryPixelMathImageFilter.h"
#include "itkSimpleBinaryPixelMathImageFilter.h"
#include "itkSimpleDistanceMapImageFilter.h"

void RegisterTests()
{
// StrategyTest01
REGISTER_TEST(TestUnaryPixelMathUC2);
REGISTER_TEST(TestUnaryPixelMathUC3);
REGISTER_TEST(TestUnaryPixelMathF2);
REGISTER_TEST(TestUnaryPixelMathF3);
REGISTER_TEST(TestBinaryPixelMathUC2);
REGISTER_TEST(TestBinaryPixelMathUC3);
REGISTER_TEST(TestBinaryPixelMathSS2);
REGISTER_TEST(TestBinaryPixelMathSS3);
REGISTER_TEST(TestBinaryPixelMathF2);
REGISTER_TEST(TestBinaryPixelMathF3);
REGISTER_TEST(TestDistanceMapF2);
REGISTER_TEST(TestDistanceMapF3);
REGISTER_TEST(TestDistanceMapD2);
REGISTER_TEST(TestDistanceMapD3);

// StrategyTest02
REGISTER_TEST(TestGradientUC2);
REGISTER_TEST(TestGradientUC3);
REGISTER_TEST(TestGradientF2);
REGISTER_TEST(TestGradientF3);
REGISTER_TEST(TestGradientMagnitudeUC2);
REGISTER_TEST(TestGradientMagnitudeUC3);
REGISTER_TEST(TestGradientMagnitudeF2);
REGISTER_TEST(TestGradientMagnitudeF3);
REGISTER_TEST(TestThresholdUC2);
REGISTER_TEST(TestThresholdUC3);
REGISTER_TEST(TestThresholdF2);
REGISTER_TEST(TestThresholdF3);

//StrategyTest03
REGISTER_TEST(TestProjectionUC3UC2);
REGISTER_TEST(TestProjectionF3F2);
REGISTER_TEST(TestMorphologyUC2);
REGISTER_TEST(TestMorphologyUC3);
REGISTER_TEST(TestMorphologyF2);
REGISTER_TEST(TestMorphologyF3);
REGISTER_TEST(TestSmoothUC2);
REGISTER_TEST(TestSmoothUC3);
REGISTER_TEST(TestSmoothF2);
REGISTER_TEST(TestSmoothF3);

//StrategyTest04
REGISTER_TEST(TestRegistrationUC2);
REGISTER_TEST(TestRegistrationUC3);
REGISTER_TEST(TestRegistrationF2);
REGISTER_TEST(TestRegistrationF3);
}

// UnaryPixelMath =========================================
template <typename TPixel, unsigned int VDimension>
int TestUnaryPixelMathTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 3 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "[InvertIntensityMaximum] ";
      std::cout << "[SigmoidAlpha] [SigmoidBeta]";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];
    itk::UnaryPixelMathStrategy Strategy =
      static_cast<itk::UnaryPixelMathStrategy>( atoi(argv[arg++]) );
    TPixel InvertIntensityMaximum = (argc > arg) ?
      (TPixel)atof( argv[arg++] ) : itk::NumericTraits<TPixel>::max();
    double SigmoidAlpha = (argc > arg) ? atof( argv[arg++] ) : 0.0;
    double SigmoidBeta = (argc > arg) ? atof( argv[arg++] ) : 1.0;

    std::cout << "Test=" << "UnaryPixelMath" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "InvertIntensityMaximum=" << (double)InvertIntensityMaximum << std::endl;
    std::cout << "SigmoidAlpha=" << SigmoidAlpha << std::endl;
    std::cout << "SigmoidBeta=" << SigmoidBeta << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::ImageFileWriter<ImageType> WriterType;
    typedef itk::SimpleUnaryPixelMathImageFilter<ImageType,ImageType> UnaryPixelMathType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename UnaryPixelMathType::Pointer filter = UnaryPixelMathType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetInvertIntensityMaximum(InvertIntensityMaximum);
    filter->SetSigmoidAlpha(SigmoidAlpha);
    filter->SetSigmoidBeta(SigmoidBeta);
    filter->SetSigmoidOutputMinimum(itk::NumericTraits<TPixel>::min());
    filter->SetSigmoidOutputMaximum(itk::NumericTraits<TPixel>::max());
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
int TestUnaryPixelMathUC2(int argc, char * argv [])
{
  return TestUnaryPixelMathTemplate<unsigned char, 2>(argc, argv);
}
int TestUnaryPixelMathUC3(int argc, char * argv [])
{
  return TestUnaryPixelMathTemplate<unsigned char, 3>(argc, argv);
}
int TestUnaryPixelMathF2(int argc, char * argv [])
{
  return TestUnaryPixelMathTemplate<float, 2>(argc, argv);
}
int TestUnaryPixelMathF3(int argc, char * argv [])
{
  return TestUnaryPixelMathTemplate<float, 3>(argc, argv);
}

// BinaryPixelMath ========================================
template <typename TPixel, unsigned int VDimension>
int TestBinaryPixelMathTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 4 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "Input1Filename Input2Filename OutputFilename Strategy ";
      std::cout << "[WeightedAddAlpha]";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* Input1Filename = argv[arg++];
    char* Input2Filename = argv[arg++];
    char* OutputFilename = argv[arg++];

     // Read strategy from command line
    itk::BinaryPixelMathStrategy Strategy =
      static_cast<itk::BinaryPixelMathStrategy>( atoi(argv[arg++]) );

    // Read other paramters
    double WeightedAddAlpha = (argc > arg) ? atof( argv[arg++] ) : 0.5;

    std::cout << "Test=" << "BinaryPixelMath" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "Input1Filename=" << Input1Filename << std::endl;
    std::cout << "Input2Filename=" << Input2Filename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "WeightedAddAlpha=" << WeightedAddAlpha << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::ImageFileWriter<ImageType> WriterType;
    typedef itk::SimpleBinaryPixelMathImageFilter<ImageType,ImageType> BinaryPixelMathType;

    // Read inputs
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(Input1Filename);
    reader->Update();
    typename ImageType::Pointer input1 = reader->GetOutput();
    input1->DisconnectPipeline();
    reader->SetFileName(Input2Filename);
    reader->Update();
    typename ImageType::Pointer input2 = reader->GetOutput();
    input2->DisconnectPipeline();

    // Filter
    typename BinaryPixelMathType::Pointer filter = BinaryPixelMathType::New();
    itk::SimpleFilterWatcher watcher(filter);
    filter->SetStrategy(Strategy);
    filter->SetInput1(input1);
    filter->SetInput2(input2);
    filter->SetWeightedAddAlpha(WeightedAddAlpha);
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
int TestBinaryPixelMathUC2(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<unsigned char, 2>(argc, argv);
}
int TestBinaryPixelMathUC3(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<unsigned char, 3>(argc, argv);
}
int TestBinaryPixelMathSS2(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<signed short, 2>(argc, argv);
}
int TestBinaryPixelMathSS3(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<signed short, 3>(argc, argv);
}
int TestBinaryPixelMathF2(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<float, 2>(argc, argv);
}
int TestBinaryPixelMathF3(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<float, 3>(argc, argv);
}

// DistanceMap ============================================
template <typename TPixel, unsigned int VDimension>
int TestDistanceMapTemplate(int argc, char * argv [])
{
  try
    {
    // Read command-line parameters
    if ( argc < 3 )
      {
      std::cout << "USAGE: " << argv[0] << " ";
      std::cout << "InputFilename OutputFilename Strategy ";
      std::cout << "[SquaredDistance] [InsideIsPositive]";
      std::cout << std::endl;
      return EXIT_FAILURE;
      }
    int arg = 1;
    char* InputFilename = argv[arg++];
    char* OutputFilename = argv[arg++];
    itk::DistanceMapStrategy Strategy =
      static_cast<itk::DistanceMapStrategy>( atoi(argv[arg++]) );
    bool SquaredDistance = (argc > arg) ? atoi( argv[arg++] ) : false;
    bool InsideIsPositive = (argc > arg) ? atoi( argv[arg++] ) : true;

    std::cout << "Test=" << "DistanceMap" << std::endl;
    std::cout << "PixelType=" << typeid(TPixel).name() << std::endl;
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "SquaredDistance=" << SquaredDistance << std::endl;
    std::cout << "InsideIsPositive=" << InsideIsPositive << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int Dimension = VDimension;
    typedef itk::Image<PixelType, Dimension> ImageType;
    typedef itk::ImageFileReader<ImageType> ReaderType;
    typedef itk::ImageFileWriter<ImageType> WriterType;
    typedef itk::SimpleDistanceMapImageFilter<ImageType,ImageType> DistanceMapType;

    // Read input
    typename ReaderType::Pointer reader = ReaderType::New();
    reader->SetFileName(InputFilename);
    reader->Update();
    typename ImageType::Pointer input = reader->GetOutput();
    input->DisconnectPipeline();

    // Filter
    typename DistanceMapType::Pointer filter = DistanceMapType::New();
    if (Strategy != itk::DistanceMapStrategySignedDanielsson)
      {
      // NOTE: SignedDanielsson strategy does not have Filter progress
      itk::SimpleFilterWatcher watcher(filter);
      }
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetUseImageSpacing(true);
    filter->SetSquaredDistance(SquaredDistance);
    try
      {
      filter->SetInsideIsPositive(InsideIsPositive);
      }
    catch (itk::ExceptionObject & err)
      {
      std::cerr << "Known exception: Danielsson does not support InsideIsPositive" << std::endl;
      std::cerr << err << std::endl; 
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
int TestDistanceMapF2(int argc, char * argv [])
{
  return TestDistanceMapTemplate<float, 2>(argc, argv);
}
int TestDistanceMapF3(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<float, 3>(argc, argv);
}
int TestDistanceMapD2(int argc, char * argv [])
{
  return TestDistanceMapTemplate<double, 2>(argc, argv);
}
int TestDistanceMapD3(int argc, char * argv [])
{
  return TestBinaryPixelMathTemplate<double, 3>(argc, argv);
}