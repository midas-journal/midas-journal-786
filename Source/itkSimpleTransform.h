/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleTransform.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleTransform_h
#define __itkSimpleTransform_h

#include "itkTransform.h"
#include "itkStrategyFactory.h"
#include "itkExceptionObject.h"

#include "itkTranslationTransform.h"
#include "itkScaleTransform.h"
#include "itkEuler2DTransform.h"
#include "itkEuler3DTransform.h"
#include "itkVersorTransform.h"
#include "itkVersorRigid3DTransform.h"
#include "itkSimilarity2DTransform.h"
#include "itkSimilarity3DTransform.h"
#include "itkAffineTransform.h"
#include "itkCenteredEuler3DTransform.h"
#include "itkCenteredRigid2DTransform.h"
#include "itkCenteredSimilarity2DTransform.h"
#include "itkCenteredAffineTransform.h"
#include "itkBSplineDeformableTransform.h"

namespace itk
{

/** List of strategies for transforms. */
enum TransformStrategy {
  TransformStrategyTranslation,
  TransformStrategyScale,
  TransformStrategyEuler2D,
  TransformStrategyEuler3D,
  TransformStrategyVersor,
  TransformStrategyVersorRigid3D,
  TransformStrategySimilarity2D,
  TransformStrategySimilarity3D,
  TransformStrategyAffine,
  //TransformStrategyCenteredEuler3D,
  //TransformStrategyCenteredRigid2D,
  //TransformStrategyCenteredSimilarity2D,
  //TransformStrategyCenteredAffine,
  TransformStrategyBSpline // TODO: Properly support BSpline
};

/** \class SimpleTransform
 * \brief Implements various transforms.
 *
 * This object is templated over the following types:
 *   TScalarType: Required scalar type used for operations
 *   NInputDimensions: Optional number of dimensions (default=3)
 *   NOutputDimensions: Optional number of dimensions (default=input)
 *
 * This object currently supports the following strategies:
 *   Translation
 *   Scale
 *   Euler2D
 *   Euler3D
 *   Versor
 *   VersorRigid3D
 *   Similarity2D
 *   Similarity3D
 *   Affine
 *   BSpline
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <class TScalarType,
          unsigned int NInputDimensions=3,
          unsigned int NOutputDimensions=NInputDimensions>
class ITK_EXPORT SimpleTransform :
  public Transform<TScalarType,NInputDimensions,NOutputDimensions>
{
public:
  /** Standard class typedefs. */
  typedef SimpleTransform Self;
  typedef Transform<TScalarType,NInputDimensions,NOutputDimensions> Superclass;
  typedef SmartPointer<Self> Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleTransform, Transform);

  /** Some typedefs. */
  itkStaticConstMacro(InputSpaceDimension, unsigned int, NInputDimensions);
  itkStaticConstMacro(OutputSpaceDimension, unsigned int, NOutputDimensions);
  typedef typename Superclass::ScalarType ScalarType;
  typedef typename Superclass::ParametersType ParametersType;
  typedef typename Superclass::ParametersValueType ParametersValueType;
  typedef typename Superclass::InputVectorType InputVectorType;
  typedef typename Superclass::OutputVectorType OutputVectorType;
  typedef typename Superclass::InputCovariantVectorType InputCovariantVectorType;
  typedef typename Superclass::OutputCovariantVectorType OutputCovariantVectorType;
  typedef typename Superclass::InputVnlVectorType InputVnlVectorType;
  typedef typename Superclass::OutputVnlVectorType OutputVnlVectorType;
  typedef typename Superclass::InputPointType InputPointType;
  typedef typename Superclass::OutputPointType OutputPointType;
  typedef typename Superclass::JacobianType JacobianType;

  /** Strategy factory typedefs */
  typedef TransformStrategy StrategyKeyType;
  typedef Superclass StrategyType;
  typedef StrategyFactory<StrategyKeyType,StrategyType> StrategyFactoryType;

  /** Get/set strategy key */
  StrategyKeyType GetStrategy()
    {
    return m_StrategyFactory.GetKey();
    }
  void SetStrategy(StrategyKeyType key)
    {
    // Check that the strategy is valid for given template parameters
    switch (key)
      {
      case TransformStrategyTranslation:
        if (Superclass::GetInputSpaceDimension() != Superclass::GetOutputSpaceDimension()) {
          itkExceptionMacro(
            "TranslationTransform expects InputSpaceDimension==OutputSpaceDimension"
          );
        }
        break;
      case TransformStrategyScale:
        if (Superclass::GetInputSpaceDimension() != Superclass::GetOutputSpaceDimension()) {
          itkExceptionMacro(
            "ScaleTransform expects InputSpaceDimension==OutputSpaceDimension"
          );
        }
        break;
      case TransformStrategyEuler2D:
        if (Superclass::GetInputSpaceDimension() != 2 || Superclass::GetOutputSpaceDimension() != 2) {
          itkExceptionMacro(
            "Euler2DTransform expects InputSpaceDimension==2 and OutputSpaceDimension==2"
          );
        }
        break;
      case TransformStrategyEuler3D:
        if (Superclass::GetInputSpaceDimension() != 3 || Superclass::GetOutputSpaceDimension() != 3) {
          itkExceptionMacro(
            "Euler3DTransform expects InputSpaceDimension==3 and OutputSpaceDimension==3"
          );
        }
        break;
      case TransformStrategyVersor:
        if (Superclass::GetInputSpaceDimension() != 3 || Superclass::GetOutputSpaceDimension() != 3) {
          itkExceptionMacro(
            "VersorTransform expects InputSpaceDimension==3 and OutputSpaceDimension==3"
          );
        }
        break;
      case TransformStrategyVersorRigid3D:
        if (Superclass::GetInputSpaceDimension() != 3 || Superclass::GetOutputSpaceDimension() != 3) {
          itkExceptionMacro(
            "VersorRigid3DTransform expects InputSpaceDimension==3 and OutputSpaceDimension==3"
          );
        }
        break;
      case TransformStrategySimilarity2D:
        if (Superclass::GetInputSpaceDimension() != 2 || Superclass::GetOutputSpaceDimension() != 2) {
          itkExceptionMacro(
            "Similarity2DTransform expects InputSpaceDimension==2 and OutputSpaceDimension==2"
          );
        }
        break;
      case TransformStrategySimilarity3D:
        if (Superclass::GetInputSpaceDimension() != 3 || Superclass::GetOutputSpaceDimension() != 3) {
          itkExceptionMacro(
            "Similarity3DTransform expects InputSpaceDimension==3 and OutputSpaceDimension==3"
          );
        }
        break;
      case TransformStrategyAffine:
        if (Superclass::GetInputSpaceDimension() != Superclass::GetOutputSpaceDimension()) {
          itkExceptionMacro(
            "TranslationTransform expects InputSpaceDimension==OutputSpaceDimension"
          );
        }
      default:
        // TODO: BSpline?
        break;
      }

    // Set the key
    m_StrategyFactory.SetKey(key);

    // Reinitialize the base class with the correct number of dimensions/parameters
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    Reinitialize(
      transform->GetOutputSpaceDimension(), // TODO: Should this be input or output?
      transform->GetNumberOfParameters()
    );

    // Mark as modified
    this->Modified();
    }

  /** Method to transform a point. */
  virtual OutputPointType TransformPoint(const InputPointType& input) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->TransformPoint(input);
    }

  /** Method to transform a vector. */
  virtual OutputVectorType TransformVector(const InputVectorType& input) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->TransformVector(input);
    }

  /** Method to transform a vnl_vector. */
  virtual OutputVnlVectorType TransformVector(const InputVnlVectorType& input) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->TransformVector(input);
    }

  /** Method to transform a CovariantVector. */
  virtual OutputCovariantVectorType TransformCovariantVector(
    const InputCovariantVectorType& input) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->TransformCovariantVector(input);
    }

  /** Method to set parameters. */
  virtual void SetParameters(const ParametersType& params)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    transform->SetParameters(params);
    }

  /** Method to set parameters. */
  virtual void SetParametersByValue(const ParametersType& params)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    transform->SetParametersByValue(params);
    }

  /** Method to get parameters. */
  virtual const ParametersType& GetParameters(void) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->GetParameters();
    }

  /** Method to set fixed parameters. */
  virtual void SetFixedParameters(const ParametersType& params)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    transform->SetFixedParameters(params);
    }

  /** Method to get fixed parameters. */
  virtual const ParametersType& GetFixedParameters(void) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->GetFixedParameters();
    }

  /** Method to get Jacobian. */
  virtual const JacobianType& GetJacobian(const InputPointType& jacobian) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->GetJacobian(jacobian);
    }

  /** Method to get number of parameters. */
  virtual unsigned int GetNumberOfParameters(void) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->GetNumberOfParameters();
    }

  /** Method to get if the transform is linear or not. */
  virtual bool IsLinear() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* transform = ptrStrategyFactory->GetStrategy();
    return transform->IsLinear();
    }

  // ** This sets matrix to identity and Offset to null. */
  virtual void SetIdentity()
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    typedef MatrixOffsetTransformBase<ScalarType, InputSpaceDimension, OutputSpaceDimension>
      MatrixOffsetTransformType;
    MatrixOffsetTransformType* matrixOffsetBase = NULL;
    switch (m_StrategyFactory.GetKey())
      {
      case TransformStrategyEuler2D:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      case TransformStrategyEuler3D:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      case TransformStrategyVersor:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      case TransformStrategyVersorRigid3D:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      case TransformStrategySimilarity2D:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      case TransformStrategySimilarity3D:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      case TransformStrategyAffine:
        matrixOffsetBase = ptrStrategyFactory->template GetConcreteStrategy
          <MatrixOffsetTransformType>(TransformStrategyAffine);
        matrixOffsetBase->SetIdentity();
        break;
      default:
        itkExceptionMacro("Current strategy does not support SetIdentity");
      }
    }

protected:
  SimpleTransform() :
    Superclass(1,1), // Use fake parameters, reinitialize when strategy is set
    m_StrategyFactory(StrategyFactoryType(this))
    {
    m_StrategyFactory.template AddStrategy<TranslationTransformType>
      ( TransformStrategyTranslation );
    m_StrategyFactory.template AddStrategy<ScaleTransformType>
      ( TransformStrategyScale );
    m_StrategyFactory.template AddStrategy<Euler2DTransformType>
      ( TransformStrategyEuler2D );
    m_StrategyFactory.template AddStrategy<Euler3DTransformType>
      ( TransformStrategyEuler3D );
    m_StrategyFactory.template AddStrategy<VersorTransformType>
      ( TransformStrategyVersor );
    m_StrategyFactory.template AddStrategy<VersorRigid3DTransformType>
      ( TransformStrategyVersorRigid3D );
    m_StrategyFactory.template AddStrategy<Similarity2DTransformType>
      ( TransformStrategySimilarity2D );
    m_StrategyFactory.template AddStrategy<Similarity3DTransformType>
      ( TransformStrategySimilarity3D );
    m_StrategyFactory.template AddStrategy<AffineTransformType>
      ( TransformStrategyAffine );
    m_StrategyFactory.template AddStrategy<BSplineTransformType>
      ( TransformStrategyBSpline );
    }
  virtual ~SimpleTransform() {};

private:
  SimpleTransform(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef TranslationTransform<ScalarType,InputSpaceDimension> // NOTE: Assumes input dim = output dim
    TranslationTransformType;
  typedef ScaleTransform<ScalarType,InputSpaceDimension> // NOTE: Assumes input dim = output dim
    ScaleTransformType;
  typedef Euler2DTransform<ScalarType> // NOTE: Assumes input/output dim = 2
    Euler2DTransformType;
  typedef Euler3DTransform<ScalarType> // NOTE: Assumes input/output dim = 3
    Euler3DTransformType;
  typedef Rigid2DTransform<ScalarType> // NOTE: Assumes input/output dim = 2
    Rigid2DTransformType;
  typedef Rigid3DTransform<ScalarType> // NOTE: Assumes input/output dim = 3
    Rigid3DTransformType;
  typedef VersorTransform<ScalarType> // NOTE: Assumes input/output dim is 3
    VersorTransformType;
  typedef VersorRigid3DTransform<ScalarType> // NOTE: Assumes input/output dim is 3
    VersorRigid3DTransformType;
  typedef Similarity2DTransform<ScalarType> // NOTE: Assumes input/output dim = 2
    Similarity2DTransformType;
  typedef Similarity3DTransform<ScalarType> // NOTE: Assumes input/output dim is 3
    Similarity3DTransformType;
  typedef AffineTransform<ScalarType,InputSpaceDimension> // NOTE: Assumes input dim = output dim
    AffineTransformType;
  //typedef CenteredEuler3DTransform<ScalarType,InputSpaceDimension,OutputSpaceDimension>
  //  CenteredEuler3DTransformType;
  //typedef CenteredRigid2DTransform<ScalarType,InputSpaceDimension,OutputSpaceDimension>
  //  CenteredRigid2DTransformType;
  //typedef CenteredSimilarity2DTransform<ScalarType,InputSpaceDimension,OutputSpaceDimension>
  //  CenteredSimilarity2DTransformType;
  //typedef CenteredAffineTransform<ScalarType,InputSpaceDimension,OutputSpaceDimension>
  //  CenteredAffineTransformType;
  typedef BSplineDeformableTransform<ScalarType,InputSpaceDimension> // NOTE: Spline order = 3
    BSplineTransformType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
