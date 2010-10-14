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

Sensor::Sensor() : Referenced(), m_timeout(5000), m_name("NO_NAME"), m_shutdown(false), m_initialized(false)
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

void Sensor::execShutdown(float time)
{
  // Call the inherited shutdown() method
  shutdown();

  SensorEventHandler::Event e(time, 
    SensorEventHandler::SHUTDOWN);

  executeEvent(e);
  m_eventHandlers.clear();
}

void Sensor::execUpdate(float time)
{
  // Call the inherited update() method
  update(time);

  SensorEventHandler::Event e(time, 
    SensorEventHandler::UPDATE);

  executeEvent(e);
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

void Sensor::registerSensorEventHandler(SensorEventHandler *eventHandler)
{ 
  if (!m_initialized)
    throw std::runtime_error("registerSensorEventHandler: init() method should be called prior to registering event handlers");

  m_eventHandlers[eventHandler] = eventHandler; 
  eventHandler->setNumberOfButtons(getNumberOfButtons());
  eventHandler->setNumberOfValuators(getNumberOfValuators());
}



bool Sensor::unregisterSensorEventHandler(SensorEventHandler *e) {
  EventMapIterator it = m_eventHandlers.find(e);
  if (it != m_eventHandlers.end()) {
    m_eventHandlers.erase(it);
    return true;
  }

  return false;
}

/// Example of updateEventHandler
void Sensor::updateEventHandlers(float time)
{

  // Update valuaters with current values
  EventMapIterator it;
  for(it = m_eventHandlers.begin(); it != m_eventHandlers.end(); it++) 
  {
    // Set the number of valuators
    it->first->setNumberOfValuators(0);

    // Set the values for each valuator
    for (unsigned int i=SensorEventHandler::VALUATOR_1; i <= getNumberOfValuators(); i++) {
      it->first->setValuatorValue((SensorEventHandler::Valuator)i, 0.0f);
    }

  }  

  // Now update the buttons
  for(it = m_eventHandlers.begin(); it != m_eventHandlers.end(); it++) 
  {  
    it->first->begin(this);

    SensorEventHandler::ButtonState state=SensorEventHandler::UP;
    
    for (unsigned int button=SensorEventHandler::BUTTON_1; button <= getNumberOfButtons(); button++) {
      it->first->setButtonState((SensorEventHandler::Button)button, state);
    }
    it->first->end();
  }


}

void Sensor::executeEvent(SensorEventHandler::Event &e)
{
  EventMapIterator it;
  for(it = m_eventHandlers.begin(); it != m_eventHandlers.end(); it++) 
  {
    it->first->begin(this);
    it->first->pushEvent(e);
    it->first->end();

  }
}
