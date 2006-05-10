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

#include <osgHaptics/VibrationForceOperator.h>
#include <hd/hd.h>
#include <vrutils/Math.h>

using namespace osgHaptics;

VibrationForceOperator::VibrationForceOperator() : ForceOperator()
{
  hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &m_amplitude);
  m_max_amplitude = m_amplitude;
  m_amplitude *= 0.75;
  m_direction.set(0,1,0);
}

void VibrationForceOperator::calculateForce(const osg::Vec3d& in, osg::Vec3d& out, double time)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  out = m_direction*sin(2*osg::PI*time*m_frequency)*m_amplitude;
}

void VibrationForceOperator::setDirection(osg::Vec3d& direction)
{
  direction.normalize();
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m_direction = direction;
}

void VibrationForceOperator::setAmplitude(double amplitude)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m_amplitude = vrutils::min(amplitude, m_max_amplitude);
}

void VibrationForceOperator::setFrequency(float f) 
{ 
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex); 
  m_frequency = f; 
}
