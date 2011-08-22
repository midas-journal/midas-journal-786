/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    itkSimpleOptimizer.h
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef __itkSimpleOptimizer_h
#define __itkSimpleOptimizer_h

#include "itkSingleValuedNonLinearOptimizer.h"
#include "itkStrategyFactory.h"
#include "itkExceptionObject.h"

#include "itkAmoebaOptimizer.h"
#include "itkConjugateGradientOptimizer.h"
#include "itkFRPROptimizer.h"
#include "itkGradientDescentOptimizer.h"
#include "itkLBFGSBOptimizer.h"
#include "itkLBFGSOptimizer.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkOnePlusOneEvolutionaryOptimizer.h"
#include "itkPowellOptimizer.h"
#include "itkRegularStepGradientDescentOptimizer.h"
#include "itkVersorRigid3DTransformOptimizer.h"
#include "itkVersorTransformOptimizer.h"

#include "itkRandomVariateGeneratorBase.h" // For OnePlusOneEvolutionaryOptimizer

namespace itk
{

/** List of strategies for optimizers. */
enum OptimizerStrategy {
  OptimizerStrategyAmoeba,
  OptimizerStrategyConjugateGradient,
  OptimizerStrategyFRPR,
  OptimizerStrategyGradientDescent,
  OptimizerStrategyLBFGSB,
  OptimizerStrategyLBFGS,
  OptimizerStrategyLevenbergMarquardt,
  OptimizerStrategyOnePlusOneEvolutionary,
  OptimizerStrategyPowell,
  OptimizerStrategyRegularStepGradientDescent,
  OptimizerStrategyVersorRigid3DTransform,
  OptimizerStrategyVersorTransform
};

/** \class SimpleOptimizer
 * \brief Implements various single-valued non-linear optimizers.
 *
 * This object currently supports the following strategies:
 *   Amoeba
 *   ConjugateGradient
 *   FRPR
 *   GradientDescent
 *   LBFGSB
 *   LBFGS
 *   LevenbergMarquardt
 *   OnePlusOneEvolutionary
 *   Powell
 *   RegularStepGradientDescent
 *   VersorRigid3DTransform
 *   VersorTransform
 *
 * This object has the following parameters:
 *   ([Type]) [Parameter] [Strategies] [Description]
 *   (bool) Maximize GradientDescent/OnePlusOneEvolutionary/RegularStepGradientDescent
 *     Specifies the optimizer should maximize the cost function.
 *   (bool) Minimize GradientDescent/OnePlusOneEvolutionary/RegularStepGradientDescent
 *     Specifies the optimizer should minimize the cost function.
 *   (unsigned int) NumberOfIterations Amoeba/ConjugateGradient/GradientDescent/
 *                                     LevenbergMarquardt/RegularStepGradientDescent
 *     Specifies the maximum number of iterations.
 *   (bool) AutomaticInitialSimplex Amoeba
 *     Specifies whether the simplex is automatically initialized.
 *   (ParametersType) InitialSimplexDelta Amoeba
 *     Specifies the initial simplex delta values.
 *   (double) ParametersConvergenceTolerance Amoeba
 *     TODO:
 *   (double) FunctionConvergenceTolerance Amoeba
 *     TODO:
 *   (bool) UseUnitLengthGradient FRPR
 *     TODO:
 *   (bool) SetToFletchReeves FRPR
 *     TODO:
 *   (double) LearningRate GradientDescent
 *     Specifies the learning rate, which decreases the step size over time.
 *   (double) ValueTolerance LevenbergMarquardt
 *     TODO:
 *   (double) GradientTolerance LevenbergMarquardt
 *     TODO:
 *   (double) EpsilonFunction LevenbergMarquardt
 *     TODO:
 *   (double) GrowthFactor OnePlusOneEvolutionary
 *     TODO:
 *   (double) ShrinkFactor OnePlusOneEvolutionary
 *     TODO:
 *   (double) InitialRadius OnePlusOneEvolutionary
 *     TODO:
 *   (double) Epsilon OnePlusOneEvolutionary
 *     TODO:
 *   (double) FrobeniusNorm OnePlusOneEvolutionary
 *     TODO:
 *   (RandomVariateGeneratorType) NormalVariateGenerator OnePlusOneEvolutionary
 *     TODO:
 *   (double) MaximumStepLength RegularStepGradientDescent
 *     TODO:
 *   (double) MinimumStepLength RegularStepGradientDescent
 *     TODO:
 *   (double) RelaxationFactor RegularStepGradientDescent
 *     TODO:
 *   (double) GradientMagnitudeTolerance RegularStepGradientDescent
 *     TODO:
 *
 * This implementation was taken from the Insight Journal:
 *     http://hdl.handle.net/10380/3248
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 * */
class ITK_EXPORT SimpleOptimizer :
  public SingleValuedNonLinearOptimizer
{
public:
  /** Standard class typedefs. */
  typedef SimpleOptimizer Self;
  typedef SingleValuedNonLinearOptimizer Superclass;
  typedef SmartPointer<Self> Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(SimpleOptimizer, SingleValuedNonLinearOptimizer);

  /** Some typedefs. */
  typedef Superclass::ParametersType ParametersType;
  typedef Superclass::ScalesType ScalesType;
  typedef Superclass::CostFunctionType CostFunctionType;
  typedef Superclass::CostFunctionPointer CostFunctionPointer;
  typedef Superclass::MeasureType MeasureType;
  typedef Superclass::DerivativeType DerivativeType;

  /** Strategy factory typedefs */
  typedef OptimizerStrategy StrategyKeyType;
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

  /**  Set the position to initialize the optimization. */
  virtual void SetInitialPosition(const ParametersType & param)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    optimizer->SetInitialPosition(param);
    }

  /** Get the position to initialize the optimization. */
  virtual const ParametersType& GetInitialPosition() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    return optimizer->GetInitialPosition();
    }

  /** Set current parameters scaling. */
  void SetScales(const ScalesType & scales)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    optimizer->SetScales(scales);
    }

  /** Get current parameters scaling. */
  virtual const ScalesType& GetScales() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    return optimizer->GetScales();
    }

  /** Get current position of the optimization. */
  virtual const ParametersType& GetCurrentPosition() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    return optimizer->GetCurrentPosition();
    }

  /** Start optimization. */
  virtual void StartOptimization()
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    optimizer->StartOptimization();
    }

  /** Get the reason for termination */
  virtual const std::string GetStopConditionDescription() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    return optimizer->GetStopConditionDescription();
    }

  /** Set the cost function. */
  virtual void SetCostFunction(CostFunctionType * costFunction)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    optimizer->SetCostFunction(costFunction);
    }

  /** Get the cost function. */
  virtual const CostFunctionType* GetCostFunction() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    return optimizer->GetCostFunction();
    }

  /** Get the cost function value at the given parameters. */
  MeasureType GetValue(const ParametersType & parameters) const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    StrategyType* optimizer = ptrStrategyFactory->GetStrategy();
    return optimizer->GetValue(parameters);
    }

  /** Get/set Maximize parameter */
  virtual const bool GetMaximize() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case OptimizerStrategyGradientDescent:
        return ptrStrategyFactory->GetConcreteStrategy<GradientDescentType>
          (OptimizerStrategyGradientDescent)->GetMaximize();
      case OptimizerStrategyOnePlusOneEvolutionary:
        return ptrStrategyFactory->GetConcreteStrategy<OnePlusOneEvolutionaryType>
          (OptimizerStrategyOnePlusOneEvolutionary)->GetMaximize();
      case OptimizerStrategyRegularStepGradientDescent:
        return ptrStrategyFactory->GetConcreteStrategy<RegularStepGradientDescentType>
          (OptimizerStrategyRegularStepGradientDescent)->GetMaximize();
      default:
        itkExceptionMacro("Current strategy does not support Maximize");
      }
    }
  virtual void SetMaximize(const bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case OptimizerStrategyGradientDescent:
        ptrStrategyFactory->GetConcreteStrategy<GradientDescentType>
          (OptimizerStrategyGradientDescent)->SetMaximize(value);
        break;
      case OptimizerStrategyOnePlusOneEvolutionary:
        ptrStrategyFactory->GetConcreteStrategy<OnePlusOneEvolutionaryType>
          (OptimizerStrategyOnePlusOneEvolutionary)->SetMaximize(value);
        break;
      case OptimizerStrategyRegularStepGradientDescent:
        ptrStrategyFactory->GetConcreteStrategy<RegularStepGradientDescentType>
          (OptimizerStrategyRegularStepGradientDescent)->SetMaximize(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support Maximize");
      }
    }

  /** Get/set Minimize parameter */
  virtual const bool GetMinimize() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case OptimizerStrategyGradientDescent:
        return ptrStrategyFactory->GetConcreteStrategy<GradientDescentType>
          (OptimizerStrategyGradientDescent)->GetMinimize();
      case OptimizerStrategyOnePlusOneEvolutionary:
        return ptrStrategyFactory->GetConcreteStrategy<OnePlusOneEvolutionaryType>
          (OptimizerStrategyOnePlusOneEvolutionary)->GetMinimize();
      case OptimizerStrategyRegularStepGradientDescent:
        return ptrStrategyFactory->GetConcreteStrategy<RegularStepGradientDescentType>
          (OptimizerStrategyRegularStepGradientDescent)->GetMinimize();
      default:
        itkExceptionMacro("Current strategy does not support Minimize");
      }
    }
  virtual void SetMinimize(const bool value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case OptimizerStrategyGradientDescent:
        ptrStrategyFactory->GetConcreteStrategy<GradientDescentType>
          (OptimizerStrategyGradientDescent)->SetMinimize(value);
        break;
      case OptimizerStrategyOnePlusOneEvolutionary:
        ptrStrategyFactory->GetConcreteStrategy<OnePlusOneEvolutionaryType>
          (OptimizerStrategyOnePlusOneEvolutionary)->SetMinimize(value);
        break;
      case OptimizerStrategyRegularStepGradientDescent:
        ptrStrategyFactory->GetConcreteStrategy<RegularStepGradientDescentType>
          (OptimizerStrategyRegularStepGradientDescent)->SetMinimize(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support Minimize");
      }
    }

  /** Get/set NumberOfIterations parameter */
  virtual const unsigned int GetNumberOfIterations() const
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      case OptimizerStrategyAmoeba:
        return ptrStrategyFactory->GetConcreteStrategy<AmoebaType>
          (OptimizerStrategyAmoeba)->GetMaximumNumberOfIterations();
      case OptimizerStrategyConjugateGradient:
        return ptrStrategyFactory->GetConcreteStrategy<ConjugateGradientType>
          (OptimizerStrategyConjugateGradient)->GetNumberOfIterations();
      case OptimizerStrategyGradientDescent:
        return ptrStrategyFactory->GetConcreteStrategy<GradientDescentType>
          (OptimizerStrategyGradientDescent)->GetNumberOfIterations();
      //case OptimizerStrategyLevenbergMarquardt:
      //  return ptrStrategyFactory->GetConcreteStrategy<LevenbergMarquardtType>
      //    (OptimizerStrategyLevenbergMarquardt)->GetNumberOfIterations();
      //case OptimizerStrategyRegularStepGradientDescent:
      //  return ptrStrategyFactory->GetConcreteStrategy<RegularStepGradientDescentType>
      //    (OptimizerStrategyRegularStepGradientDescent)->GetNumberOfIterations();
      default:
        itkExceptionMacro("Current strategy does not support NumberOfIterations");
      }
    }
  virtual void SetNumberOfIterations(const unsigned int value)
    {
    StrategyFactoryType* ptrStrategyFactory =
      const_cast<StrategyFactoryType*>(&m_StrategyFactory);
    switch (m_StrategyFactory.GetKey())
      {
      //case OptimizerStrategyAmoeba:
      //  ptrStrategyFactory->GetConcreteStrategy<AmoebaType>
      //    (OptimizerStrategyAmoeba)->SetMaximuNumberOfIterations(value);
      //  break;
      //case OptimizerStrategyConjugateGradient:
      //  ptrStrategyFactory->GetConcreteStrategy<ConjugateGradientType>
      //    (OptimizerStrategyConjugateGradient)->SetNumberOfIterations(value);
      //  break;
      case OptimizerStrategyGradientDescent:
        ptrStrategyFactory->GetConcreteStrategy<GradientDescentType>
          (OptimizerStrategyGradientDescent)->SetNumberOfIterations(value);
        break;
      case OptimizerStrategyLevenbergMarquardt:
        ptrStrategyFactory->GetConcreteStrategy<LevenbergMarquardtType>
          (OptimizerStrategyLevenbergMarquardt)->SetNumberOfIterations(value);
        break;
      case OptimizerStrategyRegularStepGradientDescent:
        ptrStrategyFactory->GetConcreteStrategy<RegularStepGradientDescentType>
          (OptimizerStrategyRegularStepGradientDescent)->SetNumberOfIterations(value);
        break;
      default:
        itkExceptionMacro("Current strategy does not support NumberOfIterations");
      }
    }

  /** Amoeba parameters. */
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, AutomaticInitialSimplex, bool
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, AutomaticInitialSimplex, bool
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, InitialSimplexDelta, ParametersType
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, InitialSimplexDelta, ParametersType
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, ParametersConvergenceTolerance, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, ParametersConvergenceTolerance, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, FunctionConvergenceTolerance, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, AmoebaType,
    Amoeba, FunctionConvergenceTolerance, double
  );

  /** ConjugateGradient parameters. */
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, ConjugateGradientType,
    ConjugateGradient, CurrentIteration, unsigned long
  );

  /** FRPR parameters. */
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, FRPRType,
    FRPR, UseUnitLengthGradient, bool
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, FRPRType,
    FRPR, UseUnitLengthGradient, bool
  );
  virtual void FRPRSetToFletchReeves( )
  {
    FRPRType* concreteObject =
      m_StrategyFactory.GetConcreteStrategy<FRPRType>(OptimizerStrategyFRPR);
    concreteObject->SetToFletchReeves( );
  }
  virtual void FRPRSetToPolakRibiere( )
  {
    FRPRType* concreteObject =
      m_StrategyFactory.GetConcreteStrategy<FRPRType>(OptimizerStrategyFRPR);
    concreteObject->SetToPolakRibiere( );
  }

  /** GradientDescent parameters. */
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, GradientDescentType,
    GradientDescent, LearningRate, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, GradientDescentType,
    GradientDescent, LearningRate, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, GradientDescentType,
    GradientDescent, CurrentIteration, unsigned long
  );

  /** LevenbergMarquardt parameters. */
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, LevenbergMarquardtType,
    LevenbergMarquardt, ValueTolerance, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, LevenbergMarquardtType,
    LevenbergMarquardt, GradientTolerance, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, LevenbergMarquardtType,
    LevenbergMarquardt, EpsilonFunction, double
  );

  /** OnePlusOneEvolutionary parameters. */
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, GrowthFactor, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, GrowthFactor, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, ShrinkFactor, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, ShrinkFactor, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, InitialRadius, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, InitialRadius, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, Epsilon, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, Epsilon, double
  );
  itkGetConcreteStrategyMacroNonTemplate(
    Optimizer, OnePlusOneEvolutionaryType,
    OnePlusOneEvolutionary, FrobeniusNorm, double
  );
  typedef Statistics::RandomVariateGeneratorBase RandomVariateGeneratorType;
  virtual void SetOnePlusOneEvolutionaryNormalVariateGenerator(
    RandomVariateGeneratorType::Pointer generator)
    {
    OnePlusOneEvolutionaryType* concreteObject = m_StrategyFactory.GetConcreteStrategy
      <OnePlusOneEvolutionaryType>(OptimizerStrategyOnePlusOneEvolutionary);
    concreteObject->SetNormalVariateGenerator(generator);
    }
  virtual void OnePlusOneEvolutionaryInitialize(
    double initialRadius, double grow = -1, double shrink = -1)
    {
    OnePlusOneEvolutionaryType* concreteObject = m_StrategyFactory.GetConcreteStrategy
      <OnePlusOneEvolutionaryType>(OptimizerStrategyOnePlusOneEvolutionary);
    concreteObject->Initialize(initialRadius,grow,shrink);
    }

  /** RegularStepGradientDescent parameters. */
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, RegularStepGradientDescentType,
    RegularStepGradientDescent, MaximumStepLength, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, RegularStepGradientDescentType,
    RegularStepGradientDescent, MinimumStepLength, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, RegularStepGradientDescentType,
    RegularStepGradientDescent, RelaxationFactor, double
  );
  itkSetConcreteStrategyMacroNonTemplate(
    Optimizer, RegularStepGradientDescentType,
    RegularStepGradientDescent, GradientMagnitudeTolerance, double
  );

protected:
  SimpleOptimizer() : m_StrategyFactory(StrategyFactoryType(this))
    {
    m_StrategyFactory.AddStrategy<AmoebaType>
      ( OptimizerStrategyAmoeba );
    m_StrategyFactory.AddStrategy<ConjugateGradientType>
      ( OptimizerStrategyConjugateGradient );
    m_StrategyFactory.AddStrategy<FRPRType>
      ( OptimizerStrategyFRPR );
    m_StrategyFactory.AddStrategy<GradientDescentType>
      ( OptimizerStrategyGradientDescent );
    m_StrategyFactory.AddStrategy<LBFGSBType>
      ( OptimizerStrategyLBFGSB );
    m_StrategyFactory.AddStrategy<LBFGSType>
      ( OptimizerStrategyLBFGS );
    m_StrategyFactory.AddStrategy<LevenbergMarquardtType>
      ( OptimizerStrategyLevenbergMarquardt );
    m_StrategyFactory.AddStrategy<OnePlusOneEvolutionaryType>
      ( OptimizerStrategyOnePlusOneEvolutionary );
    m_StrategyFactory.AddStrategy<PowellType>
      ( OptimizerStrategyPowell );
    m_StrategyFactory.AddStrategy<RegularStepGradientDescentType>
      ( OptimizerStrategyRegularStepGradientDescent );
    m_StrategyFactory.AddStrategy<VersorRigid3DTransformType>
      ( OptimizerStrategyVersorRigid3DTransform );
    m_StrategyFactory.AddStrategy<VersorTransformType>
      ( OptimizerStrategyVersorTransform );
    }
  virtual ~SimpleOptimizer() {};

private:
  SimpleOptimizer(const Self&); //purposely not implemented
  void operator=(const Self&); //purposely not implemented

  /** Strategy typedefs */
  typedef AmoebaOptimizer AmoebaType;
  typedef ConjugateGradientOptimizer ConjugateGradientType;
  typedef FRPROptimizer FRPRType;
  typedef GradientDescentOptimizer GradientDescentType;
  typedef LBFGSBOptimizer LBFGSBType;
  typedef LBFGSOptimizer LBFGSType;
  typedef LevenbergMarquardtOptimizer LevenbergMarquardtType;
  typedef OnePlusOneEvolutionaryOptimizer OnePlusOneEvolutionaryType;
  typedef PowellOptimizer PowellType;
  typedef RegularStepGradientDescentOptimizer RegularStepGradientDescentType;
  typedef VersorRigid3DTransformOptimizer VersorRigid3DTransformType;
  typedef VersorTransformOptimizer VersorTransformType;

  /* Private member variables */
  StrategyFactoryType m_StrategyFactory;
};

} // end namespace itk

#endif
