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


#include <osgHaptics/HapticDevice.h>
#include <osgHaptics/ForceEffect.h>
#include <osg/Vec3d>
#include <iostream>
#include <osg/Notify>


using namespace osgHaptics;

ForceEffect::ForceEffect(HapticDevice *device, Type type) : m_started(false), m_device(device) 
{
  assert(device);
  m_effect_id = hlGenEffects(1);
  m_type = type;
  m_device->registerForceEffect(this);
}


ForceEffect::~ForceEffect()
{
  m_device->unRegisterForceEffect(this);
  hlDeleteEffects(m_effect_id,1);
}


/// Modify the type of the effect
void ForceEffect::setType(ForceEffect::Type type)
{
  if (m_type == CALLBACK_EFFECT) {
    // Unschedule callback
    m_device->unScheduleForceEffectCallback(this);
    osg::notify(osg::WARN) << "ForceEffect::setType(): Type CALLBACK not in use" << std::endl; 
  }

  if (type == CALLBACK_EFFECT) {
    // Unschedule callback
    m_device->scheduleForceEffectCallback(this);
    osg::notify(osg::WARN) << "ForceEffect::setType(): Type CALLBACK not in use" << std::endl; 
  }

  m_type = type;
}

/// Return the type of the effect
ForceEffect::Type ForceEffect::getType() const
{
  return m_type;
}

/// Start the effect, run until stop is called
void ForceEffect::start()
{
  m_device->pushForceEffectOperation(this, START);
}

/// Start the effect, run until stop is called
void ForceEffect::executeStart()
{
  applyParameterset();
  hlStartEffect(getHLType(), m_effect_id);
  m_started = true;
}

HLenum ForceEffect::getHLType() const
{
  switch (m_type) {
    case( CALLBACK_EFFECT ):
      return HL_EFFECT_CALLBACK;
    case( CONSTANT_EFFECT ):
      return HL_EFFECT_CONSTANT;
    case( SPRING_EFFECT ):
      return HL_EFFECT_SPRING;
    case ( VISCOUS_EFFECT ):
      return HL_EFFECT_VISCOUS;
    case( FRICTION_EFFECT ): 
      return HL_EFFECT_FRICTION;
  }
  return 0;
}
void ForceEffect::applyParameterset() const 
{
  {
    FloatMap::const_iterator it;
    for(it=m_float_parameters.begin(); it != m_float_parameters.end(); it++) {
      //std::cerr << it->first << ": " << it->second << std::endl;
      hlEffectd(it->first, it->second);
    }
  }
  
  {
    Vec3Map::const_iterator it;
    for(it=m_vec3_parameters.begin(); it != m_vec3_parameters.end(); it++) {
		osg::Vec3d pos = it->second;
    if (it->first == HL_EFFECT_PROPERTY_POSITION) {
      osg::Matrix m;
      m_device->getWorldToWorkSpaceMatrix(m);
		  pos = m.preMult(pos);
    }

	  hlEffectdv(it->first, pos.ptr());
    }
  }
}

/// Stop the effect
void ForceEffect::stop()
{
  m_device->pushForceEffectOperation(this, STOP);
}

/// Stop the effect
void ForceEffect::executeStop()
{
  applyParameterset();
  hlStopEffect( m_effect_id);
  m_started = false;
}


/// Start the effect and run in duration ms.
void ForceEffect::trig()
{
  m_device->pushForceEffectOperation(this, TRIG);
}

/// Start the effect and run in duration ms.
void ForceEffect::executeTrig()
{

  applyParameterset();
  hlTriggerEffect( getHLType() );

}


/// Start the effect and run in duration ms.
void ForceEffect::update()
{
  m_device->pushForceEffectOperation(this, UPDATE);
}

void ForceEffect::executeUpdate()
{
  applyParameterset();
  hlUpdateEffect( m_effect_id );
}



// Mutators
void ForceEffect::setGain(HLdouble gain)
{
  m_float_parameters[HL_EFFECT_PROPERTY_GAIN] = gain;
}
void ForceEffect::setMagnitude(HLdouble magnitude)
{
  m_float_parameters[HL_EFFECT_PROPERTY_MAGNITUDE] = magnitude;
}

void ForceEffect::setFrequency(HLdouble f)
{
  m_float_parameters[HL_EFFECT_PROPERTY_FREQUENCY] = f;
}

void ForceEffect::setDuration(HLdouble milliseconds)
{
  m_float_parameters[HL_EFFECT_PROPERTY_FREQUENCY] = milliseconds;
}

void ForceEffect::setPosition(const osg::Vec3d& pos){
  m_vec3_parameters[HL_EFFECT_PROPERTY_POSITION] = pos;
}

void ForceEffect::setDirection(const osg::Vec3d& dir){
  m_vec3_parameters[HL_EFFECT_PROPERTY_DIRECTION] = dir;
}



// Accessors
HLdouble ForceEffect::getGain() const
{
  FloatMap::const_iterator it = m_float_parameters.find(HL_EFFECT_PROPERTY_GAIN);

  if (it == m_float_parameters.end()) {
    HLdouble f;
    hlGetEffectdv(m_effect_id, HL_EFFECT_PROPERTY_GAIN, &f);
    return f;
  }

  return it->second;
}
HLdouble ForceEffect::getMagnitude() const
{
  FloatMap::const_iterator it = m_float_parameters.find(HL_EFFECT_PROPERTY_MAGNITUDE);

  if (it == m_float_parameters.end()) {
    HLdouble f;
    hlGetEffectdv(m_effect_id, HL_EFFECT_PROPERTY_MAGNITUDE, &f);
    return f;
  }

  return it->second;
}
HLdouble ForceEffect::getFrequency() const
{
  FloatMap::const_iterator it = m_float_parameters.find(HL_EFFECT_PROPERTY_FREQUENCY);

  if (it == m_float_parameters.end()) {
    HLdouble f;
    hlGetEffectdv(m_effect_id, HL_EFFECT_PROPERTY_FREQUENCY, &f);
    return f;
  }

  return it->second;
}
HLdouble ForceEffect::getDuration() const
{
  FloatMap::const_iterator it = m_float_parameters.find(HL_EFFECT_PROPERTY_DURATION);

  if (it == m_float_parameters.end()) {
    HLdouble f;
    hlGetEffectdv(m_effect_id, HL_EFFECT_PROPERTY_DURATION, &f);
    return f;
  }

  return it->second;
}
osg::Vec3d ForceEffect::getPosition() const
{
  Vec3Map::const_iterator it = m_vec3_parameters.find(HL_EFFECT_PROPERTY_POSITION);

  if (it == m_vec3_parameters.end()) {
    osg::Vec3d v;
    hlGetEffectdv(m_effect_id, HL_EFFECT_PROPERTY_POSITION, v.ptr());
    return v;
  }

  return it->second;
}

osg::Vec3d ForceEffect::getDirection() const
{
  Vec3Map::const_iterator it = m_vec3_parameters.find(HL_EFFECT_PROPERTY_DIRECTION);

  if (it == m_vec3_parameters.end()) {
    osg::Vec3d v;
    hlGetEffectdv(m_effect_id, HL_EFFECT_PROPERTY_DIRECTION, v.ptr());
    return v;
  }

  return it->second;
}
