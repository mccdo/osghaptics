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

#include <osgSensor/SensorEventHandler.h>
#include <iostream>
#include <osg/Notify>
#include <osgSensor/bitoperators.h>
using namespace osgSensor;

SensorEventHandler::SensorEventHandler(const std::string& name) : 
    m_initialized(false), 
    m_buttonMask(0), 
    m_useIndividualEvents(false),  
    m_eventType(TYPE_NONE), 
    m_numberOfButtons(0),
    m_name(name), 
    m_beginCalled(false),
    m_eventMode(QUEUE_EVENTS), 
    m_sensor(0L)
{
  m_activationMask.set(); // Enable all activation events

  m_buttonState.resize(MAX_NUM_BUTTONS, UP);
  m_valuatorValues.resize(MAX_NUM_VALUATORS, 0.0f);

}


SensorEventHandler::~SensorEventHandler()
{
  while(m_eventQueue.size())
    m_eventQueue.pop();

  m_buttonState.clear();
  m_valuatorValues.clear();
}

void SensorEventHandler::begin(Sensor *sensor)
{
  m_sensor = sensor;

  m_beginCalled=true;
  m_buttonMask=0;
  m_eventType=TYPE_NONE;
  m_button = BUTTON_NONE;

//  for(int i=0; i < MAX_NUM_BUTTONS; i++)
//    m_buttonState[i] = STATE_NONE;
}


void SensorEventHandler::end()
{
  if (!m_beginCalled) {
    std::cerr << "SensorEventHandler::end(): Should always be preceded by a call to begin()" << std::endl;
    return;
  }
  m_beginCalled = false;


  while(m_eventQueue.size()) {
    Event e = m_eventQueue.front();

    m_eventQueue.pop();
    
    // Should we trigger this eventtype?
    if (!m_activationMask.test(e.eventType))
      continue;

   
    m_buttonState[e.button] = e.buttonState;
    m_eventType = e.eventType;
    m_button = e.button;

    
    // Do the callback
    this->operator ()(m_eventType, e.time);
  }
}




void 
SensorEventHandler::pushEvent(float t, EventType type, Button b, ButtonState state)
{
  Event e(t,type,b,state);


  if (type == BUTTON) {
    if (state == DOWN)
      osgSensor::setBit(m_buttonMask, b);
    else if (state == UP)
      osgSensor::clearBit(m_buttonMask, b);
  }

  // Should we queue up the events, or should we queue up the events?
  // This also means that we will or up the button state so when the event finally occurs
  if ( getEventMode() == QUEUE_EVENTS ) {
    m_eventQueue.push(e);
  }
  else {  // Immediate mode, execute the event immediately
    m_eventQueue.push(e);
    end();
  }
}
float SensorEventHandler::getValuatorValue(Valuator v) const
{
  if ( (unsigned int)v < m_valuatorValues.size() && v < MAX_NUM_VALUATORS)
  {
    return m_valuatorValues[v];
  }
  else {
    osg::notify(osg::WARN) << "SensorEventHandler::setValuatorValue" << "Invalid valuator specified: " << v << std::endl;
    return 0.0f;
  }
}

void SensorEventHandler::setValuatorValue(Valuator v, float value)
{
  unsigned int vv = v;
  vv--;
  if (vv < m_valuatorValues.size() && vv < MAX_NUM_VALUATORS )
  {
    m_valuatorValues[vv] = value;
  }
  else {
    osg::notify(osg::WARN) << "Joystick::setValuatorValue() " << "Invalid valuator specified: " << v << std::endl;
    return;
  }
}


void SensorEventHandler::setButtonState(Button b, ButtonState state)
{ 
  if ((unsigned int)b < m_buttonState.size() )
    m_buttonState[b] = state; 
  else 
    osg::notify(osg::WARN) << " SensorEventHandler::setButtonState(): Invalid button specified (" << b << ")" << std::endl;
}

SensorEventHandler::ButtonState SensorEventHandler::getButtonState(Button b) const
{ 
    // Unused warning
  //unsigned int i = m_buttonState.size();

  if ((unsigned int)b < m_buttonState.size() ) {
    // If b== BUTTON_NONE, then return the state of the button that generated a button event.
    // Now if we are calling this method without a button event, well, then we will get an empty event...

    if (b == BUTTON_NONE)
      return m_buttonState[getButton()];

    return m_buttonState[b]; 
  }
  else 
    osg::notify(osg::WARN) << " SensorEventHandler::getButtonState(): Invalid button specified (" << b << ")" << std::endl;
  return STATE_NONE;
}



/// Returns true if button b is available
bool SensorEventHandler::queryButton(Button b)
{
  return true;
}

/// Returns true if valuator v is available
bool SensorEventHandler::queryValuator(Valuator v)
{
  return true;
}


void SensorEventHandler::setActivationBit(EventType eventType, bool enabled_flag)
{
  if (enabled_flag)
    m_activationMask[eventType]=1;
  else
    m_activationMask[eventType]=0;
}

bool SensorEventHandler::getActivationBit(EventType eventType) const
{
  return m_activationMask.test(eventType);
}


void SensorEventHandler::setActivationMask(unsigned int mask)
{

  const unsigned int uint_size = sizeof(unsigned int)*8;
  m_activationMask.reset();
  for(unsigned int bit=0; mask && bit < (m_activationMask.size()-1) && bit < uint_size; bit++) {

    if (osgSensor::isBitSet(mask, bit))
      m_activationMask[bit+1]=1;

    osgSensor::clearBit(mask,bit);

  }
}
