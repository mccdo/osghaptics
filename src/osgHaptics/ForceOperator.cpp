/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Haptic Library
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

#include <osgHaptics/ForceOperator.h>


using namespace osgHaptics;

void ForceOperator::update()
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  if (!m_trigged)
    return;

  // Calculate how many milliseconds since the trig method was called
  osg::Timer_t now = osg::Timer::instance()->tick();
  double delta = osg::Timer::instance()->delta_m(m_start, now);
  if (delta >= m_duration)
  {
    m_enabled = false;
    m_trigged=false;
    m_duration=0;
    m_start=0;
  }
}


void ForceOperator::trig(unsigned int milliseconds_duration)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m_enabled = true;
  m_duration =  milliseconds_duration;
  m_start = osg::Timer::instance()->tick();
  m_trigged = true;
}

void ForceOperator::setWorldToWorkSpaceMatrix(const osg::Matrix& m)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m_world_to_workspace_matrix = m;
  m_workspace_to_world_matrix = m.inverse(m);
}

void ForceOperator::getWorldToWorkSpaceMatrix(osg::Matrix& m) const
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m = m_world_to_workspace_matrix;
}
