/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkStrategyFactory.h,v $
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __itkStrategyFactory_h
#define __itkStrategyFactory_h

#include "itkMacro.h"
#include "itkExceptionObject.h"
#include "itkCreateObjectFunction.h"
#include "itkStrategyCommand.h"

namespace itk
{

/** Helpful macros for setting parameters. */
#define itkSetConcreteStrategyMacro(enumPrefix,concreteType,concreteName,parameterName,parameterType) \
  virtual void Set##concreteName##parameterName(const parameterType _arg) \
  { \
    itkDebugMacro("setting " #concreteName#parameterName " to " << _arg); \
    concreteType* concreteObject = \
      m_StrategyFactory.template GetConcreteStrategy<concreteType>(enumPrefix##Strategy##concreteName); \
    concreteObject->Set##parameterName(_arg); \
    this->Modified(); \
  }
#define itkSetConcreteStrategyMacroNonTemplate(enumPrefix,concreteType,concreteName,parameterName,parameterType) \
  virtual void Set##concreteName##parameterName(const parameterType _arg) \
  { \
    itkDebugMacro("setting " #concreteName#parameterName " to " << _arg); \
    concreteType* concreteObject = \
      m_StrategyFactory.GetConcreteStrategy<concreteType>(enumPrefix##Strategy##concreteName); \
    concreteObject->Set##parameterName(_arg); \
    this->Modified(); \
  }
  
/** Helpful macros for getting parameters. */
#define itkGetConcreteStrategyMacro(enumPrefix,concreteType,concreteName,parameterName,parameterType) \
  virtual parameterType Get##concreteName##parameterName() \
  { \
    itkDebugMacro("returning " << #concreteName#parameterName); \
    concreteType* concreteObject = \
    m_StrategyFactory.template GetConcreteStrategy<concreteType>(enumPrefix##Strategy##concreteName); \
    return concreteObject->Get##parameterName(); \
  }
#define itkGetConcreteStrategyMacroNonTemplate(enumPrefix,concreteType,concreteName,parameterName,parameterType) \
  virtual parameterType Get##concreteName##parameterName() \
  { \
    itkDebugMacro("returning " << #concreteName#parameterName); \
    concreteType* concreteObject = \
    m_StrategyFactory.GetConcreteStrategy<concreteType>(enumPrefix##Strategy##concreteName); \
    return concreteObject->Get##parameterName(); \
  }
  
/** \class StrategyFactory
 * \brief A factory for obtaining an algorithm strategy.
 *
 * See http://en.wikipedia.org/wiki/Strategy_pattern and
 * http://en.wikipedia.org/wiki/Factory_pattern.
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
template <typename TKey, typename TStrategy>
class ITKCommon_EXPORT StrategyFactory
{
public:
  /** Constructor and copy constructor. */
  StrategyFactory();
  StrategyFactory(TStrategy* owner);
  StrategyFactory(const StrategyFactory&) {};

  /** Virtual destructor needed  */
  virtual ~StrategyFactory();

  /** Useful typedefs. */
  typedef StrategyFactory Self;
  typedef TKey KeyType;
  typedef TStrategy StrategyType;
  typedef typename StrategyType::Pointer StrategyPointer;

  virtual const char *GetNameOfClass() const
    {return "StrategyFactory";}

  /** Clear all products from the factory. */
  void Clear();

  /** Add a concrete strategy to the factory. */
  template<typename TConcreteStrategy>
  void AddStrategy(KeyType key)
  {
    m_CreationMap[key] = CreateObjectFunction<TConcreteStrategy>::New();
  }

  /** Get/set the current key. */
  KeyType GetKey() const;
  void SetKey(KeyType key);

  /** Get the strategy using the current key.
   *  Throws an exception if the given concrete strategy type can not be created.
   */
  TStrategy* GetStrategy();

  /** Get the strategy using the given key.
   *  Throws an exception if the given concrete strategy type can not be created.
   */
  TStrategy* GetStrategy(KeyType key);

  /** Get the concrete strategy using the current key.
   *  Throws an exception if the given concrete strategy type can not be created.
   */
  template<typename TConcreteStrategy>
  TConcreteStrategy* GetConcreteStrategy()
  {
    return GetConcreteStrategy<TConcreteStrategy>(m_Key);
  }

  /** Get the concrete strategy using the given key.
   *  Throws an exception if the given concrete strategy type can not be created.
   */
  template<typename TConcreteStrategy>
  TConcreteStrategy* GetConcreteStrategy(KeyType key)
  {
    TStrategy* strategy = GetStrategy(key);
    TConcreteStrategy* concreteStrategy = dynamic_cast<TConcreteStrategy*>(strategy);
    if (concreteStrategy != NULL)
      {
      return concreteStrategy;
      }
    itkExceptionMacro("Concrete strategy type does not match key");
  }

private:
  typedef std::map<KeyType, CreateObjectFunctionBase::Pointer> CreationMapType;
  typedef std::map<KeyType, StrategyPointer> InstanceMapType;
  void operator=(const StrategyFactory&);
  CreationMapType m_CreationMap;
  InstanceMapType m_InstanceMap;
  KeyType m_Key;
  StrategyType* m_Owner;
};

} // end namespace itk

#if ITK_TEMPLATE_TXX
#include "itkStrategyFactory.txx"
#endif

#endif
