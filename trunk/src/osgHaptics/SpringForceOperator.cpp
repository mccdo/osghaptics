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

#include <osgHaptics/SpringForceOperator.h>

#include <hd/hd.h>
#include <vrutils/Math.h>
#include <iostream>
#include <osg/io_utils>

using namespace osgHaptics;

SpringForceOperator::SpringForceOperator() : ForceOperator(), m_stiffness(1), m_fadein_ms(0),
  m_fade(false), m_fade_started(false), m_fade_start_time(0), m_damping(0.001)
{
  m_max_stiffness = 200;
  //hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &m_max_stiffness);
}

void SpringForceOperator::calculateForce(const osg::Vec3d& in, osg::Vec3d& out, double time)
{

  osg::Vec3d current_pos;
  hdGetDoublev(HD_CURRENT_POSITION, current_pos.ptr());

  // Calculate the damping force -b*v
  osg::Vec3d curr_vel;
  hdGetDoublev(HD_LAST_VELOCITY, curr_vel.ptr());  
  osg::Vec3 damp_force = -curr_vel*m_damping;

  // Lock the ForceOperator
  m_mutex.lock();

  osg::Vec3 world_pos = m_workspace_to_world_matrix.preMult(current_pos);
  osg::Vec3 m_current_position=m_target_position;
  if (m_fade) {
    
    // We are fading, store the current time as the start of the fading.
    if (!m_fade_started) {
      m_fade_started = true;
      m_fade_start_time = time; // Start time for interpolation
      m_position = world_pos; // Save current position
    }
    double s = (time-m_fade_start_time)/(m_fadein_ms*0.001);
    m_current_position = vrutils::mix(m_position, m_target_position, s);

    // Are we there yet?
    if (time > m_fade_start_time+m_fadein_ms)
      m_fade = false;
  }

  // Set this if we are fading the Spring out to true, and we will disable the spring
  bool should_disable = false;
  m_current_fade = 1;
  // We are fading stiffness
  if (m_fade_force) {
    
    // We are fading, store the current time as the start of the fading.
    if ( !m_fade_force_started ) {
      m_fade_force_started = true;
      m_fade_force_start_time = time;
    }
    
    double s = (time-m_fade_force_start_time)/(m_fade_force_time*0.001);
    
    // Are we fading up or down
    if (m_fade_force_up)
      m_current_fade = vrutils::mix(0.0, 1.0, s);
    else
      m_current_fade = vrutils::mix(1.0, 0.0, s);

    // Are we there yet ;-)
    if (time >= (m_fade_force_start_time+m_fade_force_time*0.001)) {
      m_fade_force = false;
    }

    // Are we fading out and done with it, disable the Spring
    if (!m_fade_force && !m_fade_force_up) {
      should_disable = true;
    }
  }

  // Calculate the springforce as k*x
  out = (m_current_position - world_pos)*m_stiffness;

  // Transform the force back to workspace coordinates
  osg::Quat q;
  q.set(m_world_to_workspace_matrix);
  osg::Matrix m;
  m.set(q);
  
  // Add the viscosity force to
  out = m.preMult(out) + damp_force;
  out *= m_current_fade;

  // Unlock the ForceOperator
  m_mutex.unlock();

  if (should_disable) {
    setEnable(false);
  }

}

void SpringForceOperator::setPosition(osg::Vec3d& position, double fadein_ms)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  const double epsilon=1E-10;
  // If fadein_ms == 0, then dont interpolate position
  if (abs(fadein_ms) < epsilon) {
    m_position = position;
	m_current_position = position;
    m_target_position = position;
    m_fade = false;
    m_fadein_ms = 0;
    m_fade_started = false; 
  }

  // Ok do interpolation of position
  else {
    // The position we are aiming for
    // We will interpolate from m_position to m_target_position in fadein_ms milliseconds
    m_target_position = position;
    m_fade = true;
    m_fade_started = false; // We havent started the interpolation yet
    m_fadein_ms = fadein_ms;

    // We should store the current time...
    // What happens if this FOrceOperator is disabled?
  }
}

void SpringForceOperator::setStiffness(double stiffness)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m_stiffness = vrutils::min(stiffness, m_max_stiffness);
}

void SpringForceOperator::setDamping(double damping)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
  m_damping = damping;
}

void SpringForceOperator::setEnable(bool flag)
{
  m_mutex.lock();
  m_fade_force = false;
  m_fade_force_start_time = 0;
  m_fade_force_started = false;
  m_mutex.unlock();

  ForceOperator::setEnable(flag);
}

void SpringForceOperator::setEnable(bool flag, double fade_ms)
{
  m_mutex.lock();

  /* 
    If there is a fade down already initiated, then we must let that finish first.
    Then this call will be silently ignored.

  */
  if (m_fade_force_started && m_fade_force && !flag && !m_fade_force_up && fade_ms > 1) {
    m_mutex.unlock();
    return;
  }

  m_fade_force = true; // We will interpolate
  m_fade_force_up = flag; // We will fade up/down
  m_fade_force_time = fade_ms;  // The time it will take
  m_fade_force_start_time = 0; // Current time not set
  m_fade_force_started = false; // We havent started yet
  m_mutex.unlock();

  // If we are fading out the spring, then setEnable(false) will be called
  // automatically in a while
  if (flag)
    ForceOperator::setEnable(flag);

}
