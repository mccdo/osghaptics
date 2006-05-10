/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Sensor Library
* Copyright (C) 2006 VRlab, Umeå University
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
*/
#include <osgSensor/Event.h>
#include <OpenThreads/ScopedLock>

using namespace osgSensor;

Event::Event() 
{

	m_signaled = false;
	m_count = 0;
}

Event::~Event()
{
}

void Event::signal(void)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(m_mutex);
  m_signaled = true;
  m_count++;
  m_condition.broadcast();
}

void Event::reset(void)
{
  m_signaled = false;  
}

bool Event::wait(void)
{
  return wait(TIMEOUT_INFINITE);
}

bool Event::wait(unsigned long timeout)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> lock(m_mutex);
  bool result = true;
  long count = m_count;
  while(!m_signaled && m_count == count)
  {
    return m_condition.wait(&m_mutex, timeout)==0;
  }

  return true;
}
