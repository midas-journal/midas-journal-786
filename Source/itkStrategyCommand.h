/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkStrategyCommand.h,v $
  Language:  C++
  Date:      $Date$
  Version:   $Revision$

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __itkStrategyCommand_h
#define __itkStrategyCommand_h

#include "itkMacro.h"
#include "itkCommand.h"
#include "itkProcessObject.h"

namespace itk
{
/** \class StrategyCommand
 * \brief A command used to pass events between the
 *        concrete strategy and the strategy/owner.
 *
 * \author Dan Mueller, dan[dot]muel[at]gmail[dot]com
 *
 */
class ITKCommon_EXPORT StrategyCommand : public Command
{
public:
  /** Standard class typedefs. */
  typedef StrategyCommand     Self;
  typedef SmartPointer<Self>  Pointer;
  
  /** Method for creation through the object factory. */
  itkNewMacro(Self);
  
  /** Run-time type information (and related methods). */
  itkTypeMacro(StrategyCommand,Command);

  /** Set the owner object to signal when events are raised. */
  itkSetObjectMacro(Owner, Object);

  /** Called when the native ITK event is raised. */
  void Execute(itk::Object *caller, const itk::EventObject& eventObj)
  {
    Execute(static_cast<const itk::Object*>(caller), eventObj);
  }

  void Execute(const itk::Object *caller, const itk::EventObject& eventObj)
  {
    // Pass the event to the owner
    if (m_Owner != NULL)
      {
      // Set progress
      if (ProgressEvent().CheckEvent(&eventObj))
        {
        const ProcessObject* process = dynamic_cast<const ProcessObject*>(caller);
        ProcessObject::Pointer ownerAsProcess = dynamic_cast<ProcessObject*>(m_Owner);
        if ((process != NULL) && ownerAsProcess.IsNotNull())
          {
          ownerAsProcess->SetProgress(process->GetProgress());
          }
        }
      // Pass along event
      m_Owner->InvokeEvent(eventObj);
      }
  }

private:
  Object* m_Owner;

};

} // end namespace itk

#endif