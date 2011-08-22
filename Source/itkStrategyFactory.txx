/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkStrategyFactory.txx,v $
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __itkStrategyFactory_txx
#define __itkStrategyFactory_txx

#include "itkStrategyFactory.h"

namespace itk
{

template<typename TKey, typename TStrategy>
StrategyFactory<TKey, TStrategy>
::StrategyFactory() : m_Owner(NULL)
{
  m_Key = static_cast<TKey>(0);
}

template<typename TKey, typename TStrategy>
StrategyFactory<TKey, TStrategy>
::StrategyFactory(TStrategy* owner) : m_Owner(owner)
{
  m_Key = static_cast<TKey>(0);
}

template<typename TKey, typename TStrategy>
StrategyFactory<TKey, TStrategy>
::~StrategyFactory()
{
  // Remove all observers
  typename InstanceMapType::iterator it;
  for (it = m_InstanceMap.begin(); it != m_InstanceMap.end(); ++it)
    {
    it->second->RemoveAllObservers();
    }

  // Clear maps
  m_InstanceMap.clear();
  m_CreationMap.clear();
}

template<typename TKey, typename TStrategy>
void
StrategyFactory<TKey, TStrategy>
::Clear()
{
  m_CreationMap.clear();
  m_InstanceMap.clear();
}

template<typename TKey, typename TStrategy>
TKey
StrategyFactory<TKey, TStrategy>
::GetKey() const
{
  return m_Key;
}

template<typename TKey, typename TStrategy>
void
StrategyFactory<TKey, TStrategy>
::SetKey(KeyType key)
{
  m_Key = key;
}

template<typename TKey, typename TStrategy>
TStrategy*
StrategyFactory<TKey, TStrategy>
::GetStrategy()
{
  return GetStrategy(m_Key);
}

template<typename TKey, typename TStrategy>
TStrategy*
StrategyFactory<TKey, TStrategy>
::GetStrategy(KeyType key)
{
  StrategyType* instance = m_InstanceMap[key];
  if (instance != NULL)
    {
    return instance;
    }
  else
    {
    CreateObjectFunctionBase* createFunction =
      m_CreationMap[key];
    if (createFunction != NULL)
      {
      LightObject::Pointer newInstance =
        createFunction->CreateObject();
      StrategyType* newInstanceAsStrategy =
        dynamic_cast<StrategyType*>(newInstance.GetPointer());
      if (newInstanceAsStrategy != NULL)
        {
        // Add an observer to pass events to the owner
        StrategyCommand::Pointer observer = StrategyCommand::New();
        observer->SetOwner(m_Owner);
        newInstanceAsStrategy->AddObserver(AnyEvent(), observer);

        // Save the instance in the map, then return
        m_InstanceMap[key] = newInstanceAsStrategy;
        return newInstanceAsStrategy;
        }
      }
    }
  itkExceptionMacro("Strategy type for given key does not exist");
}

} // end namespace itk

#endif
