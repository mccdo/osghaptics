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


#include <osgSensor/SensorMgr.h>
#include <osg/ref_ptr>

using namespace osgSensor;

SensorMgr::SensorMgr() : m_shutdown(false)
{
  init();
}

bool SensorMgr::init()
{
  return true;
}


SensorMgr * SensorMgr::instance()
{
  static osg::ref_ptr <SensorMgr> s_SensorMgr = new SensorMgr;
  return s_SensorMgr.get();
}

// iterate over all sensors and call update on them
void SensorMgr::update()
{
  SensorMap::iterator it = m_sensors.begin();
  for(;it != m_sensors.end(); it++) {
    it->second->update();
  }
}

// shutdown all registrated sensors
void SensorMgr::shutdown()
{
  m_shutdown = true;

  SensorMap::iterator it = m_sensors.begin();
  for(;it != m_sensors.end(); it++) {
    Sensor *sensor =  it->second.get(); 
    sensor->getNumberOfSensors();
    sensor->shutdown();
  }

  m_sensors.clear();
}

Sensor *SensorMgr::find(const std::string& name)
{
  SensorMap::iterator it= m_sensors.find(name);

  if (it != m_sensors.end())
    return it->second.get();

  return 0L;
}

SensorMgr::~SensorMgr()
{
  if (!m_shutdown)
    shutdown();
}

void SensorMgr::registerSensor(osgSensor::Sensor *sensor)
{
  assert(sensor);
  m_sensors.insert(std::make_pair(sensor->getName(), sensor));
}

bool SensorMgr::unRegisterSensor(osgSensor::Sensor *sensor)
{
  if (m_shutdown)
    return false;

  SensorMap::iterator it = m_sensors.begin();
  for(; it != m_sensors.end(); it++) {
    if (it->second.get() == sensor) {
      m_sensors.erase(it);
      return true;
    }
  }
  return false;
}
