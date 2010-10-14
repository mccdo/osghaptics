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

#include <osgHaptics/ContactState.h>
#include <osgHaptics/HapticDevice.h>
#include <ostream>
using namespace osgHaptics;

  namespace osgHaptics {
    std::ostream& operator<<(std::ostream& os, osgHaptics::ContactState& state)
    {
      //   os << "Shape : " << state.m_shape->getName() << std::endl;
      os << "Event : " << state.m_event << std::endl;
      os << "Time  : " << state.m_time << std::endl;
      os << "Pos   : " << state.m_position << std::endl;
      os << "Normal: " << state.m_normal << std::endl;
      os << "Force : " << state.m_force << std::endl;
      os << "Torque: " << state.m_torque << std::endl;
      os << "L.Vel : " << state.m_velocity << std::endl;

      return os;
    }

  }

void ContactState::set(ContactEvent event, Shape* shape, HapticDevice *device, double time)
{
  m_event = event;
  m_shape = shape;
  m_time = time;

}


void ContactState::set(ContactEvent event, Shape* shape, HapticDevice *device, const osg::Vec3& pos, const osg::Vec3& normal, double time)
{
  m_event = event;
  m_shape = shape;
  m_time = time;
  m_normal = normal;
  m_position = pos;

  // Device velocity, force and torque
  m_velocity = device->getLinearVelocity();
  m_force = device->getForce();
  m_torque = device->getTorque();
}

