/*=========================================================================

Program:   Insight Segmentation & Registration Toolkit
Module:    $RCSfile: main.cxx,v $
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
#define NO_TESTING

#include "itkWin32Header.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkSimpleFilterWatcher.h"
#include "itkSimpleProjectionImageFilter.h"

// Projection =============================================
template <typename TPixel, unsigned int VDimension>
int mainProjection(int argc, char * argv [])
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
    std::cout << "Dimension=" << VDimension << std::endl;
    std::cout << "InputFilename=" << InputFilename << std::endl;
    std::cout << "OutputFilename=" << OutputFilename << std::endl;
    std::cout << "Strategy=" << Strategy << std::endl;
    std::cout << "ProjectionDimension=" << ProjectionDimension << std::endl;

    // Helpful typedefs
    typedef TPixel PixelType;
    static const unsigned int InputDimension = VDimension;
    static const unsigned int OutputDimension = VDimension - 1;
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
    filter->SetStrategy(Strategy);
    filter->SetInput(input);
    filter->SetProjectionDimension(ProjectionDimension);
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

int main(int argc, char *argv[])
{
  try
    {
    mainProjection<unsigned char, 3>(argc, argv);
    }
  catch (itk::ExceptionObject &ex)
    {
    std::cerr << ex << std::endl;
    return EXIT_FAILURE;
    }
  return EXIT_SUCCESS;
}
