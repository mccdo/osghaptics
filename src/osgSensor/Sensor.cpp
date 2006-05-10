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
#include <osgSensor/Sensor.h>
#include <osgSensor/SensorMgr.h>
using namespace osgSensor;

Sensor::Sensor() : Referenced(), m_timeout(5000), m_name("NO_NAME"), m_shutdown(false)
{
  SensorMgr::instance()->registerSensor(this); 
}

Sensor::Sensor(const std::string& name) : Referenced(), m_name(name), m_timeout(5000), m_shutdown(false)
{
  SensorMgr::instance()->registerSensor(this); 
}


Sensor::~Sensor() 
{ 
  if (m_shutdown)
    return;

  m_shutdown = true;
  m_name = ""; 
  SensorMgr::instance()->unRegisterSensor(this); 
}


int DummySensor::read(unsigned int sensor_no, osg::Vec3& p, osg::Quat& q, unsigned long timeout ) 
{
  p.set(1,2,3);
  q.set(1,2,3,4);
  return 1;
}

/// Returns the sensor information
int DummySensor::read(osg::Vec3& p, osg::Quat& q) {
  p.set(1,2,3);
  q.set(1,2,3,4);
  return 1;
}

/// Read the first sensor (or the only) and set the transformation matrix
int DummySensor::read(osg::Matrix& matrix ) { 
  matrix.set(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16);
  return 1;
}
