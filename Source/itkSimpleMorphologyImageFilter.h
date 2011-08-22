/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleMorphologyImageFilter.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleMorphologyImageFilter_h
#define __itkSimpleMorphologyImageFilter_h

#include "itkNumericTraits.h"
#include "itkImageToImageFilter.h"
#include "itkStrategyFactory.h"
#include "itkFlatStructuringElement.h"

#include "itkGrayscaleDilateImageFilter.h"
#include "itkGrayscaleErodeImageFilter.h"
#include "itkGrayscaleMorphologicalClosingImageFilter.h"
#include "itkGrayscaleMorphologicalOpeningImageFilter.h"
#include "itkBinaryDilateImageFilter.h"
#include "itkBinaryErodeImageFilter.h"
#include "itkBinaryMorphologicalClosingImageFilter.h"
#include "itkBinaryMorphologicalOpeningImageFilter.h"
#include "itkClosingByReconstructionImageFilter.h"
#include "itkOpeningByReconstructionImageFilter.h"
#include "itkBlackTopHatImageFilter.h"
#include "itkWhiteTopHatImageFilter.h"

#include "itkGrayscaleGeodesicDilateImageFilter.h"
#include "itkGrayscaleGeodesicErodeImageFilter.h"
#include "itkReconstructionByDilationImageFilter.h"
#include "itkReconstructionByErosionImageFilter.h"
#include "itkAreaClosingImageFilter.h"
#include "itkAreaOpeningImageFilter.h"
#include "itkGrayscaleConnectedClosingImageFilter.h"
#include "itkGrayscaleConnectedOpeningImageFilter.h"
#include "itkHConcaveImageFilter.h"
#include "itkHConvexImageFilter.h"
#include "itkHMaximaImageFilter.h"
#include "itkHMinimaImageFilter.h"
#include "itkRegionalMaximaImageFilter.h"
#include "itkRegionalMinimaImageFilter.h"

namespace itk
{

/** List of strategies for morphology image filtering. */
enum MorphologyStrategy {
  // Requires stucturing element
  MorphologyStrategyDilate,
  MorphologyStrategyErode,
  MorphologyStrategyClose,
  MorphologyStrategyOpen,
  MorphologyStrategyBinaryDilate,
  MorphologyStrategyBinaryErode,
  MorphologyStrategyBinaryClose,
  MorphologyStrategyBinaryOpen,
  MorphologyStrategyCloseByReconstruction,
  MorphologyStrategyOpenByReconstruction,
  MorphologyStrategyBlackTopHat,
  MorphologyStrategyWhiteTopHat,
  // No stucturing element
  MorphologyStrategyGeodesicDilate,
  MorphologyStrategyGeodesicErode,
  MorphologyStrategyDilateByReconstruction,
  MorphologyStrategyErodeByReconstruction,
  MorphologyStrategyAreaClose,
  MorphologyStrategyAreaOpen,
  MorphologyStrategyConnectedClose,
  MorphologyStrategyConnectedOpen,
  MorphologyStrategyHConcave,
  MorphologyStrategyHConvex,
  MorphologyStrategyHMaxima,
  MorphologyStrategyHMinima,
  MorphologyStrategyRegionalMaxima,
  MorphologyStrategyRegionalMinima
};

/** \class SimpleMorphologyImageFilter
 * \brief Implements various mathematical morphology image filters.
 *
 * This filter is templated over the following types:
 *   TInputImage: Required input image type
 *   ToutputImage: Optional output image type
 *   TKernel: Optional structuring element type
 *
 * This filter expects the following inputs:
 *   Input1: The image to apply morphology filter
 *   Input2: Optional "mask" image for reconstruction strategies.
 *           See itkReconstructionImageFilter.h for full details.
 *
 * This filter produces the following outputs:
 *   Output1: The morphological filtered image
 *
 * This filter currently supports the following strategies which
 * utilize a structuring element (TKernel):
 *   Dilate: out = max_kernel(in)
 *   Erode: out = min_kernel(in)
 *   Close: out = erode(dilate(in))
 *   Open: out = dilate(erode(in))
 *   BinaryDilate: dilation optimized for binary image
 *   BinaryErode: erosion optimized for binary image
 *   BinaryClose: closing optimized for binary image
 *   BinaryOpen: opening optimized for binary image
 *   CloseByReconstruction: out = erodeByReconstruction(in, dilate(in))
 *   OpenByReconstruction: out = dilateByReconstruction(in, erode(in))
 *   BlackTopHat: out = close(in) - in
 *   WhiteTopHat: out = in - open(in)
 *
 * This filter currently supports the following strategies which
 * do NOT need a structuring element:
 *   GeodesicDilate: The input "marker" image is dilated using an elementary
 *     structuring element (neighborhood radius one, face connected neighbors).
 *     The resulting image is then compared with the "mask" image. The output
 *     image is the pixelwise minimum of dilated marker image and mask image.
 *   GeodesicErode: The input "marker" image is eroded using an elementary
 *     structuring element (neighborhood radius one, face connected neighbors).
 *     The resulting image is then compared with the "mask" image. The output
 *     image is the pixelwise minimum of eroded marker image and mask image.
 *   DilateByReconstruction: dilation of the input "marker" image with respect
 *     to the mask image iterated until stability. See [1], [2], and [3].
 *   ErodeByReconstruction: erosion of the input "marker" image with respect
 *     to the mask image iterated until stability. See [1], [2], and [3].
 *   AreaClose: When applied to binary images, area opening has the effect
 *     of removing blobs smaller than given attribute value. When applied to
 *     greyscale images, area opening has the effect of trimming peaks based
 *     on area while leaving the rest of the image unchanged.
 *   AreaOpen: When applied to binary images, area opening has the effect
 *     of removing blobs smaller than given attribute value. When applied to
 *     greyscale images, area opening has the effect of trimming peaks based
 *     on area while leaving the rest of the image unchanged.
 *   ConnectedClose: Enhance dark objects which are surrounded by a brigher
 *     object. The dark object is specfied by a seed.
 *   ConnectedOpen: Enhance bright objects which are surrounded by a darker
 *     object. The bright object is specfied by a seed.
 *   HConcave: Extract local minima that are more than h intensity units below
 *     the (local) background. This has the effect of extracting objects that
 *     are darker than the background by at least h intensity units.
 *   HConvex: Extract local maxima that are more than h intensity units above
 *     the (local) background. This has the effect of extracting objects that
 *     are brighter than the background by at least h intensity units.
 *   HMaxima: Suppresses local maxima that are less than h intensity units
 *      above the (local) background. This has the effect of smoothing over
 *      the "high" parts of the noise in the image without smoothing over
 *      large changes in intensity (region boundaries).
 *   HMinima: Suppresses local minima that are less than h intensity units
 *      below the (local) background. This has the effect of smoothing over
 *      the "low" parts of the noise in the image without smoothing over
 *      large changes in intensity (region boundaries).
 *   RegionalMaxima: Extract flat zones surounded by pixels of lower value.
 *   RegionalMinima: Extract flat zones surounded by pixels of greater value.
 *
 * This filter has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (KernelType) Kernel Dilate/Erode/Close/Open/
 *                       BinaryDilate/BinaryErode/BinaryClose/BinaryDilate/
 *                       CloseByReconstruction/OpenByReconstruction/
 *                       BlackTopHat/WhiteTopHat
 *     Specifies the structuring element.
 *   (bool) SafeBorder Close/Open/BinaryClose/BlackTopHat/WhiteTopHat
 *     A border is temporarily added to the image to prevent border artefacts.
 *   (bool) FullyConnected CloseByReconstruction/OpenByReconstruction/
 *                         GeodesicDilate/GeodesicErode/AreaClose/AreaOpen/
 *                         ConnectedClose/ConnectedOpen/HConcave/HConvex/
 *                         HMaxima/HMinima/RegionalMaxima/RegionalMinima
 *     Specifies whether connected components are defined strictly by face
 *     connectivity or by face+edge+vertex connectivity.
 *   (InputPixelType) ForegroundValue BinaryDilate/BinaryErode/
 *                                    BinaryClose/BinaryOpen
 *     Specify the value in the image considered the "foreground".
 *   (double) AreaLambda AreaClose/AreaOpen
 *     Specify the area threshold for the operation.
 *   (InputIndexType) ConnectedSeed ConnectedClose/ConnectedOpen
 *     Specify the seed used to determine the dark/bright objects.
 *   (InputPixelType) Height HConcave/HConvex/HMaxima/HMinima
 *     Specify the pixel intensity "height" for the operation.
 *
 * References:
 * [1] Soille, "Morphological Image Analysis: Principles and Applications",
 *     Second Edition, Springer, 2003
 * [2] Robinson and Whelan, "Efficient Morphological Reconstruction:
 *     A Downhill Filter", Pattern Recognition Letters, Vol. 25, No. 15,
 *     2004, pp. 1759--1767.
 * [3] Vincent, "Morphological Grayscale Reconstruction in Image Analysis:
 *     Applications and Efficient Algorithms", IEEE Transactions on Image
 *     Processing, Vol. 2, 1993
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 * */
template <class TInputImage,
          class TOutputImage=TInputImage,
          class TKernel=FlatStructuringElement< ::itk::GetImageDimension<TInputImage>::ImageDimension > >
class ITK_EXPORT SimpleMorphologyImageFilter :
  public ImageToImageFilter<TInputImage,TOutputImage>
{
public:
  /** Standard class typedefs. */
  typedef SimpleMorphologyImageFilter                   Self;
  typedef ImageToImageFilter<TInputImage,TOutputImage>  Superclass;
  typedef SmartPointer<Self>                            Pointer;
  typedef SmartPointer<const Self>                      ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleMorphologyImageFilter, ImageToImageFilter);

  /** Some typedefs. */
  typedef TInputImage                              InputImageType;
  typedef typename    InputImageType::ConstPointer InputImagePointer;
  typedef typename    InputImageType::RegionType   InputRegionType;
  typedef typename    InputImageType::IndexType    InputIndexType;
  typedef typename    InputImageType::PixelType    InputPixelType;
  typedef TOutputImage                             OutputImageType;
  typedef typename     OutputImageType::Pointer    OutputImagePointer;
  typedef typename     OutputImageType::RegionType OutputRegionType;
  typedef typename     OutputImageType::PixelType  OutputPixelType;
  typedef TKernel                                  KernelType;

  /** Strategy factory typedefs */
  typedef MorphologyStrategy StrategyKeyType;
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

  /** Get/set Kernel parameter */
  virtual const KernelType& GetKernel() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyDilate:
        return ptrStrategyFactory->template GetConcreteStrategy<DilateType>
          (MorphologyStrategyDilate)->GetKernel();
      case MorphologyStrategyErode:
        return ptrStrategyFactory->template GetConcreteStrategy<ErodeType>
          (MorphologyStrategyErode)->GetKernel();
      case MorphologyStrategyClose:
        return ptrStrategyFactory->template GetConcreteStrategy<CloseType>
          (MorphologyStrategyClose)->GetKernel();
      case MorphologyStrategyOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<OpenType>
          (MorphologyStrategyOpen)->GetKernel();
      case MorphologyStrategyBinaryDilate:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryDilateType>
          (MorphologyStrategyBinaryDilate)->GetKernel();
      case MorphologyStrategyBinaryErode:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryErodeType>
          (MorphologyStrategyBinaryErode)->GetKernel();
      case MorphologyStrategyBinaryClose:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryCloseType>
          (MorphologyStrategyBinaryClose)->GetKernel();
      case MorphologyStrategyBinaryOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryOpenType>
          (MorphologyStrategyBinaryOpen)->GetKernel();
      case MorphologyStrategyCloseByReconstruction:
        return ptrStrategyFactory->template GetConcreteStrategy<CloseByReconstructionType>
          (MorphologyStrategyCloseByReconstruction)->GetKernel();
      case MorphologyStrategyOpenByReconstruction:
        return ptrStrategyFactory->template GetConcreteStrategy<OpenByReconstructionType>
          (MorphologyStrategyOpenByReconstruction)->GetKernel();
      case MorphologyStrategyBlackTopHat:
        return ptrStrategyFactory->template GetConcreteStrategy<BlackTopHatType>
          (MorphologyStrategyBlackTopHat)->GetKernel();
      case MorphologyStrategyWhiteTopHat:
        return ptrStrategyFactory->template GetConcreteStrategy<WhiteTopHatType>
          (MorphologyStrategyWhiteTopHat)->GetKernel();
      default:
        itkExceptionMacro("Current strategy does not support Kernel");
      }
    }
  virtual void SetKernel(const KernelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyDilate:
        ptrStrategyFactory->template GetConcreteStrategy<DilateType>
          (MorphologyStrategyDilate)->SetKernel(value);
        break;
      case MorphologyStrategyErode:
        ptrStrategyFactory->template GetConcreteStrategy<ErodeType>
          (MorphologyStrategyErode)->SetKernel(value);
        break;
      case MorphologyStrategyClose:
        ptrStrategyFactory->template GetConcreteStrategy<CloseType>
          (MorphologyStrategyClose)->SetKernel(value);
        break;
      case MorphologyStrategyOpen:
        ptrStrategyFactory->template GetConcreteStrategy<OpenType>
          (MorphologyStrategyOpen)->SetKernel(value);
        break;
      case MorphologyStrategyBinaryDilate:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryDilateType>
          (MorphologyStrategyBinaryDilate)->SetKernel(value);
        break;
      case MorphologyStrategyBinaryErode:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryErodeType>
          (MorphologyStrategyBinaryErode)->SetKernel(value);
        break;
      case MorphologyStrategyBinaryClose:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryCloseType>
          (MorphologyStrategyBinaryClose)->SetKernel(value);
        break;
      case MorphologyStrategyBinaryOpen:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryOpenType>
          (MorphologyStrategyBinaryOpen)->SetKernel(value);
        break;
      case MorphologyStrategyBlackTopHat:
        ptrStrategyFactory->template GetConcreteStrategy<BlackTopHatType>
          (MorphologyStrategyBlackTopHat)->SetKernel(value);
        break;
      case MorphologyStrategyWhiteTopHat:
        ptrStrategyFactory->template GetConcreteStrategy<WhiteTopHatType>
          (MorphologyStrategyWhiteTopHat)->SetKernel(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support Kernel");
      }
    }

  /** Get/set SafeBorder parameter */
  virtual const bool GetSafeBorder() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      //case MorphologyStrategyDilate:
      //  return ptrStrategyFactory->template GetConcreteStrategy<DilateType>
      //    (MorphologyStrategyDilate)->GetSafeBorder();
      //case MorphologyStrategyErode:
      //  return ptrStrategyFactory->template GetConcreteStrategy<ErodeType>
      //    (MorphologyStrategyErode)->GetSafeBorder();
      case MorphologyStrategyClose:
        return ptrStrategyFactory->template GetConcreteStrategy<CloseType>
          (MorphologyStrategyClose)->GetSafeBorder();
      case MorphologyStrategyOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<OpenType>
          (MorphologyStrategyOpen)->GetSafeBorder();
      //case MorphologyStrategyBinaryDilate:
      //  return ptrStrategyFactory->template GetConcreteStrategy<BinaryDilateType>
      //    (MorphologyStrategyBinaryDilate)->GetSafeBorder();
      //case MorphologyStrategyBinaryErode:
      //  return ptrStrategyFactory->template GetConcreteStrategy<BinaryErodeType>
      //    (MorphologyStrategyBinaryErode)->GetSafeBorder();
      case MorphologyStrategyBinaryClose:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryCloseType>
          (MorphologyStrategyBinaryClose)->GetSafeBorder();
      //case MorphologyStrategyBinaryOpen:
      //  return ptrStrategyFactory->template GetConcreteStrategy<BinaryOpenType>
      //    (MorphologyStrategyBinaryOpen)->GetSafeBorder();
      //case MorphologyStrategyCloseByReconstruction:
      //  return ptrStrategyFactory->template GetConcreteStrategy<CloseByReconstructionType>
      //    (MorphologyStrategyCloseByReconstruction)->GetSafeBorder();
      //case MorphologyStrategyOpenByReconstruction:
      //  return ptrStrategyFactory->template GetConcreteStrategy<OpenByReconstructionType>
      //    (MorphologyStrategyOpenByReconstruction)->GetSafeBorder();
      case MorphologyStrategyBlackTopHat:
        return ptrStrategyFactory->template GetConcreteStrategy<BlackTopHatType>
          (MorphologyStrategyBlackTopHat)->GetSafeBorder();
      case MorphologyStrategyWhiteTopHat:
        return ptrStrategyFactory->template GetConcreteStrategy<WhiteTopHatType>
          (MorphologyStrategyWhiteTopHat)->GetSafeBorder();
      default:
        itkExceptionMacro("Current strategy does not support SafeBorder");
      }
    }
  virtual void SetSafeBorder(const bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      //case MorphologyStrategyDilate:
      //  ptrStrategyFactory->template GetConcreteStrategy<DilateType>
      //    (MorphologyStrategyDilate)->SetSafeBorder(value);
      //  break;
      //case MorphologyStrategyErode:
      //  ptrStrategyFactory->template GetConcreteStrategy<ErodeType>
      //    (MorphologyStrategyErode)->SetSafeBorder(value);
      //  break;
      case MorphologyStrategyClose:
        ptrStrategyFactory->template GetConcreteStrategy<CloseType>
          (MorphologyStrategyClose)->SetSafeBorder(value);
        break;
      case MorphologyStrategyOpen:
        ptrStrategyFactory->template GetConcreteStrategy<OpenType>
          (MorphologyStrategyOpen)->SetSafeBorder(value);
        break;
      //case MorphologyStrategyBinaryDilate:
      //  ptrStrategyFactory->template GetConcreteStrategy<BinaryDilateType>
      //    (MorphologyStrategyBinaryDilate)->SetSafeBorder(value);
      //  break;
      //case MorphologyStrategyBinaryErode:
      //  ptrStrategyFactory->template GetConcreteStrategy<BinaryErodeType>
      //    (MorphologyStrategyBinaryErode)->SetSafeBorder(value);
      //  break;
      case MorphologyStrategyBinaryClose:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryCloseType>
          (MorphologyStrategyBinaryClose)->SetSafeBorder(value);
        break;
      //case MorphologyStrategyBinaryOpen:
      //  ptrStrategyFactory->template GetConcreteStrategy<BinaryOpenType>
      //    (MorphologyStrategyBinaryOpen)->SetSafeBorder(value);
      //  break;
      case MorphologyStrategyBlackTopHat:
        ptrStrategyFactory->template GetConcreteStrategy<BlackTopHatType>
          (MorphologyStrategyBlackTopHat)->SetSafeBorder(value);
        break;
      case MorphologyStrategyWhiteTopHat:
        ptrStrategyFactory->template GetConcreteStrategy<WhiteTopHatType>
          (MorphologyStrategyWhiteTopHat)->SetSafeBorder(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support SafeBorder");
      }
    }

  /** Get/set FullyConnected parameter */
  virtual const bool GetFullyConnected() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyCloseByReconstruction:
        return ptrStrategyFactory->template GetConcreteStrategy<CloseByReconstructionType>
          (MorphologyStrategyCloseByReconstruction)->GetFullyConnected();
      case MorphologyStrategyOpenByReconstruction:
        return ptrStrategyFactory->template GetConcreteStrategy<OpenByReconstructionType>
          (MorphologyStrategyOpenByReconstruction)->GetFullyConnected();
      case MorphologyStrategyGeodesicDilate:
        return ptrStrategyFactory->template GetConcreteStrategy<GeodesicDilateType>
          (MorphologyStrategyGeodesicDilate)->GetFullyConnected();
      case MorphologyStrategyGeodesicErode:
        return ptrStrategyFactory->template GetConcreteStrategy<GeodesicErodeType>
          (MorphologyStrategyGeodesicErode)->GetFullyConnected();
      case MorphologyStrategyAreaClose:
        return ptrStrategyFactory->template GetConcreteStrategy<AreaCloseType>
          (MorphologyStrategyAreaClose)->GetFullyConnected();
      case MorphologyStrategyAreaOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<AreaOpenType>
          (MorphologyStrategyAreaOpen)->GetFullyConnected();
      case MorphologyStrategyConnectedClose:
        return ptrStrategyFactory->template GetConcreteStrategy<ConnectedCloseType>
          (MorphologyStrategyConnectedClose)->GetFullyConnected();
      case MorphologyStrategyConnectedOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<ConnectedOpenType>
          (MorphologyStrategyConnectedOpen)->GetFullyConnected();
      case MorphologyStrategyHConcave:
        return ptrStrategyFactory->template GetConcreteStrategy<HConcaveType>
          (MorphologyStrategyHConcave)->GetFullyConnected();
      case MorphologyStrategyHConvex:
        return ptrStrategyFactory->template GetConcreteStrategy<HConvexType>
          (MorphologyStrategyHConvex)->GetFullyConnected();
      case MorphologyStrategyHMaxima:
        return ptrStrategyFactory->template GetConcreteStrategy<HMaximaType>
          (MorphologyStrategyHMaxima)->GetFullyConnected();
      case MorphologyStrategyHMinima:
        return ptrStrategyFactory->template GetConcreteStrategy<HMinimaType>
          (MorphologyStrategyHMinima)->GetFullyConnected();
      case MorphologyStrategyRegionalMaxima:
        return ptrStrategyFactory->template GetConcreteStrategy<RegionalMaximaType>
          (MorphologyStrategyRegionalMaxima)->GetFullyConnected();
      case MorphologyStrategyRegionalMinima:
        return ptrStrategyFactory->template GetConcreteStrategy<RegionalMinimaType>
          (MorphologyStrategyRegionalMinima)->GetFullyConnected();
      default:
        itkExceptionMacro("Current strategy does not support FullyConnected");
      }
    }
  virtual void SetFullyConnected(const bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyCloseByReconstruction:
        ptrStrategyFactory->template GetConcreteStrategy<CloseByReconstructionType>
          (MorphologyStrategyCloseByReconstruction)->SetFullyConnected(value);
        break;
      case MorphologyStrategyOpenByReconstruction:
        ptrStrategyFactory->template GetConcreteStrategy<OpenByReconstructionType>
          (MorphologyStrategyOpenByReconstruction)->SetFullyConnected(value);
        break;
      case MorphologyStrategyGeodesicDilate:
        ptrStrategyFactory->template GetConcreteStrategy<GeodesicDilateType>
          (MorphologyStrategyGeodesicDilate)->SetFullyConnected(value);
        break;
      case MorphologyStrategyGeodesicErode:
        ptrStrategyFactory->template GetConcreteStrategy<GeodesicErodeType>
          (MorphologyStrategyGeodesicErode)->SetFullyConnected(value);
        break;
      case MorphologyStrategyAreaClose:
        ptrStrategyFactory->template GetConcreteStrategy<AreaCloseType>
          (MorphologyStrategyAreaClose)->SetFullyConnected(value);
        break;
      case MorphologyStrategyAreaOpen:
        ptrStrategyFactory->template GetConcreteStrategy<AreaOpenType>
          (MorphologyStrategyAreaOpen)->SetFullyConnected(value);
        break;
      case MorphologyStrategyConnectedClose:
        ptrStrategyFactory->template GetConcreteStrategy<ConnectedCloseType>
          (MorphologyStrategyConnectedClose)->SetFullyConnected(value);
        break;
      case MorphologyStrategyConnectedOpen:
        ptrStrategyFactory->template GetConcreteStrategy<ConnectedOpenType>
          (MorphologyStrategyConnectedOpen)->SetFullyConnected(value);
        break;
      case MorphologyStrategyHConcave:
        ptrStrategyFactory->template GetConcreteStrategy<HConcaveType>
          (MorphologyStrategyHConcave)->SetFullyConnected(value);
        break;
      case MorphologyStrategyHConvex:
        ptrStrategyFactory->template GetConcreteStrategy<HConvexType>
          (MorphologyStrategyHConvex)->SetFullyConnected(value);
        break;
      case MorphologyStrategyHMaxima:
        ptrStrategyFactory->template GetConcreteStrategy<HMaximaType>
          (MorphologyStrategyHMaxima)->SetFullyConnected(value);
        break;
      case MorphologyStrategyHMinima:
        ptrStrategyFactory->template GetConcreteStrategy<HMinimaType>
          (MorphologyStrategyHMinima)->SetFullyConnected(value);
        break;
      case MorphologyStrategyRegionalMaxima:
        ptrStrategyFactory->template GetConcreteStrategy<RegionalMaximaType>
          (MorphologyStrategyRegionalMaxima)->SetFullyConnected(value);
        break;
      case MorphologyStrategyRegionalMinima:
        ptrStrategyFactory->template GetConcreteStrategy<RegionalMinimaType>
          (MorphologyStrategyRegionalMinima)->SetFullyConnected(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support FullyConnected");
      }
    }

  /** Get/set ForegroundValue parameter */
  virtual const InputPixelType GetForegroundValue() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyBinaryDilate:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryDilateType>
          (MorphologyStrategyBinaryDilate)->GetForegroundValue();
      case MorphologyStrategyBinaryErode:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryErodeType>
          (MorphologyStrategyBinaryErode)->GetForegroundValue();
      case MorphologyStrategyBinaryClose:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryCloseType>
          (MorphologyStrategyBinaryClose)->GetForegroundValue();
      case MorphologyStrategyBinaryOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<BinaryOpenType>
          (MorphologyStrategyBinaryOpen)->GetForegroundValue();
      default:
        itkExceptionMacro("Current strategy does not support ForegroundValue");
      }
    }
  virtual void SetForegroundValue(const InputPixelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyBinaryDilate:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryDilateType>
          (MorphologyStrategyBinaryDilate)->SetForegroundValue(value);
        break;
      case MorphologyStrategyBinaryErode:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryErodeType>
          (MorphologyStrategyBinaryErode)->SetForegroundValue(value);
        break;
      case MorphologyStrategyBinaryClose:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryCloseType>
          (MorphologyStrategyBinaryClose)->SetForegroundValue(value);
        break;
      case MorphologyStrategyBinaryOpen:
        ptrStrategyFactory->template GetConcreteStrategy<BinaryOpenType>
          (MorphologyStrategyBinaryOpen)->SetForegroundValue(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support ForegroundValue");
      }
    }

  /** Get/set AreaLambda parameter */
  virtual const double GetAreaLambda() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyAreaClose:
        return ptrStrategyFactory->template GetConcreteStrategy<AreaCloseType>
          (MorphologyStrategyAreaClose)->GetLambda();
      case MorphologyStrategyAreaOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<AreaOpenType>
          (MorphologyStrategyAreaOpen)->GetLambda();
      default:
        itkExceptionMacro("Current strategy does not support AreaLambda");
      }
    }
  virtual void SetAreaLambda(const double value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyAreaClose:
        ptrStrategyFactory->template GetConcreteStrategy<AreaCloseType>
          (MorphologyStrategyAreaClose)->SetLambda(value);
        break;
      case MorphologyStrategyAreaOpen:
        ptrStrategyFactory->template GetConcreteStrategy<AreaOpenType>
          (MorphologyStrategyAreaOpen)->SetLambda(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support AreaLambda");
      }
    }

  /** Get/set ConnectedSeed parameter */
  virtual const InputIndexType GetConnectedSeed() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyConnectedClose:
        return ptrStrategyFactory->template GetConcreteStrategy<ConnectedCloseType>
          (MorphologyStrategyConnectedClose)->GetSeed();
      case MorphologyStrategyConnectedOpen:
        return ptrStrategyFactory->template GetConcreteStrategy<ConnectedOpenType>
          (MorphologyStrategyConnectedOpen)->GetSeed();
      default:
        itkExceptionMacro("Current strategy does not support ConnectedSeed");
      }
    }
  virtual void SetConnectedSeed(const InputIndexType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyConnectedClose:
        ptrStrategyFactory->template GetConcreteStrategy<ConnectedCloseType>
          (MorphologyStrategyConnectedClose)->SetSeed(value);
        break;
      case MorphologyStrategyConnectedOpen:
        ptrStrategyFactory->template GetConcreteStrategy<ConnectedOpenType>
          (MorphologyStrategyConnectedOpen)->SetSeed(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support ConnectedSeed");
      }
    }

  /** Get/set Height parameter */
  virtual const InputPixelType GetHeight() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyHConcave:
        return ptrStrategyFactory->template GetConcreteStrategy<HConcaveType>
          (MorphologyStrategyHConcave)->GetHeight();
      case MorphologyStrategyHConvex:
        return ptrStrategyFactory->template GetConcreteStrategy<HConvexType>
          (MorphologyStrategyHConvex)->GetHeight();
      case MorphologyStrategyHMaxima:
        return ptrStrategyFactory->template GetConcreteStrategy<HMaximaType>
          (MorphologyStrategyHMaxima)->GetHeight();
      case MorphologyStrategyHMinima:
        return ptrStrategyFactory->template GetConcreteStrategy<HMinimaType>
          (MorphologyStrategyHMinima)->GetHeight();
      default:
        itkExceptionMacro("Current strategy does not support Height");
      }
    }
  virtual void SetHeight(const InputPixelType value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case MorphologyStrategyHConcave:
        ptrStrategyFactory->template GetConcreteStrategy<HConcaveType>
          (MorphologyStrategyHConcave)->SetHeight(value);
        break;
      case MorphologyStrategyHConvex:
        ptrStrategyFactory->template GetConcreteStrategy<HConvexType>
          (MorphologyStrategyHConvex)->SetHeight(value);
        break;
      case MorphologyStrategyHMaxima:
        ptrStrategyFactory->template GetConcreteStrategy<HMaximaType>
          (MorphologyStrategyHMaxima)->SetHeight(value);
        break;
      case MorphologyStrategyHMinima:
        ptrStrategyFactory->template GetConcreteStrategy<HMinimaType>
          (MorphologyStrategyHMinima)->SetHeight(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support Height");
      }
    }

protected:
  SimpleMorphologyImageFilter() : m_StrategyFactory( StrategyFactoryType(this) )
  {
    this->SetNumberOfRequiredInputs(1); // TODO: Some strategies require 2...

    m_StrategyFactory.template AddStrategy<DilateType>
      ( MorphologyStrategyDilate );
    m_StrategyFactory.template AddStrategy<ErodeType>
      ( MorphologyStrategyErode );
    m_StrategyFactory.template AddStrategy<CloseType>
      ( MorphologyStrategyClose );
    m_StrategyFactory.template AddStrategy<OpenType>
      ( MorphologyStrategyOpen );
    m_StrategyFactory.template AddStrategy<BinaryDilateType>
      ( MorphologyStrategyBinaryDilate );
    m_StrategyFactory.template AddStrategy<BinaryErodeType>
      ( MorphologyStrategyBinaryErode );
    m_StrategyFactory.template AddStrategy<BinaryCloseType>
      ( MorphologyStrategyBinaryClose );
    m_StrategyFactory.template AddStrategy<BinaryOpenType>
      ( MorphologyStrategyBinaryOpen );
    m_StrategyFactory.template AddStrategy<BlackTopHatType>
      ( MorphologyStrategyBlackTopHat );
    m_StrategyFactory.template AddStrategy<WhiteTopHatType>
      ( MorphologyStrategyWhiteTopHat );

    m_StrategyFactory.template AddStrategy<GeodesicDilateType>
      ( MorphologyStrategyGeodesicDilate );
    m_StrategyFactory.template AddStrategy<GeodesicErodeType>
      ( MorphologyStrategyGeodesicErode );
    m_StrategyFactory.template AddStrategy<DilateByReconstructionType>
      ( MorphologyStrategyDilateByReconstruction );
    m_StrategyFactory.template AddStrategy<ErodeByReconstructionType>
      ( MorphologyStrategyErodeByReconstruction );
    m_StrategyFactory.template AddStrategy<CloseByReconstructionType>
      ( MorphologyStrategyCloseByReconstruction );
    m_StrategyFactory.template AddStrategy<OpenByReconstructionType>
      ( MorphologyStrategyOpenByReconstruction );
    m_StrategyFactory.template AddStrategy<AreaCloseType>
      ( MorphologyStrategyAreaClose );
    m_StrategyFactory.template AddStrategy<AreaOpenType>
      ( MorphologyStrategyAreaOpen );
    m_StrategyFactory.template AddStrategy<ConnectedCloseType>
      ( MorphologyStrategyConnectedClose );
    m_StrategyFactory.template AddStrategy<ConnectedOpenType>
      ( MorphologyStrategyConnectedOpen );
    m_StrategyFactory.template AddStrategy<HConcaveType>
      ( MorphologyStrategyHConcave );
    m_StrategyFactory.template AddStrategy<HConvexType>
      ( MorphologyStrategyHConvex );
    m_StrategyFactory.template AddStrategy<HMaximaType>
      ( MorphologyStrategyHMaxima );
    m_StrategyFactory.template AddStrategy<HMinimaType>
      ( MorphologyStrategyHMinima );
    m_StrategyFactory.template AddStrategy<RegionalMaximaType>
      ( MorphologyStrategyRegionalMaxima );
    m_StrategyFactory.template AddStrategy<RegionalMinimaType>
      ( MorphologyStrategyRegionalMinima );
  }
  virtual ~SimpleMorphologyImageFilter() {};

  void GenerateData()
  {
    StrategyType* filter = m_StrategyFactory.GetStrategy();
    filter->SetInput( this->GetInput() );
    filter->GraftOutput( this->GetOutput() );
    filter->Update();
    this->GraftOutput( filter->GetOutput() );
  }

private:
  SimpleMorphologyImageFilter(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef GrayscaleDilateImageFilter<InputImageType,OutputImageType,KernelType>
    DilateType;
  typedef GrayscaleErodeImageFilter<InputImageType,OutputImageType,KernelType>
    ErodeType;
  typedef GrayscaleMorphologicalClosingImageFilter<InputImageType,OutputImageType,KernelType>
    CloseType;
  typedef GrayscaleMorphologicalOpeningImageFilter<InputImageType,OutputImageType,KernelType>
    OpenType;
  typedef BinaryDilateImageFilter<InputImageType,OutputImageType,KernelType>
    BinaryDilateType;
  typedef BinaryErodeImageFilter<InputImageType,OutputImageType,KernelType>
    BinaryErodeType;
  typedef BinaryMorphologicalClosingImageFilter<InputImageType,OutputImageType,KernelType>
    BinaryCloseType;
  typedef BinaryMorphologicalOpeningImageFilter<InputImageType,OutputImageType,KernelType>
    BinaryOpenType;
  typedef ClosingByReconstructionImageFilter<InputImageType,OutputImageType,KernelType>
    CloseByReconstructionType;
  typedef OpeningByReconstructionImageFilter<InputImageType,OutputImageType,KernelType>
    OpenByReconstructionType;
  typedef BlackTopHatImageFilter<InputImageType,OutputImageType,KernelType>
    BlackTopHatType;
  typedef WhiteTopHatImageFilter<InputImageType,OutputImageType,KernelType>
    WhiteTopHatType;

  typedef GrayscaleGeodesicDilateImageFilter<InputImageType,OutputImageType>
    GeodesicDilateType;
  typedef GrayscaleGeodesicErodeImageFilter<InputImageType,OutputImageType>
    GeodesicErodeType;
  typedef ReconstructionByDilationImageFilter<InputImageType,OutputImageType>
    DilateByReconstructionType;
  typedef ReconstructionByErosionImageFilter<InputImageType,OutputImageType>
    ErodeByReconstructionType;
  typedef AreaClosingImageFilter<InputImageType,OutputImageType>
    AreaCloseType;
  typedef AreaOpeningImageFilter<InputImageType,OutputImageType>
    AreaOpenType;
  typedef GrayscaleConnectedClosingImageFilter<InputImageType,OutputImageType>
    ConnectedCloseType;
  typedef GrayscaleConnectedOpeningImageFilter<InputImageType,OutputImageType>
    ConnectedOpenType;
  typedef HConcaveImageFilter<InputImageType,OutputImageType>
    HConcaveType;
  typedef HConvexImageFilter<InputImageType,OutputImageType>
    HConvexType;
  typedef HMaximaImageFilter<InputImageType,OutputImageType>
    HMaximaType;
  typedef HMinimaImageFilter<InputImageType,OutputImageType>
    HMinimaType;
  typedef RegionalMaximaImageFilter<InputImageType,OutputImageType>
    RegionalMaximaType;
  typedef RegionalMinimaImageFilter<InputImageType,OutputImageType>
    RegionalMinimaType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
