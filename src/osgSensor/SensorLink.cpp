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


#include <osgSensor/SensorLink.h>
#include <osg/Notify>


using namespace osgSensor;

SensorLink::SensorLink(OsgSensor *sensor) : m_sensor(sensor), m_enable(true)
{
  if (!m_sensor)
    osg::notify(osg::FATAL) << "SensorLink::SensorLink(): sensor should not be null" << __FILE__ <<":" <<  __LINE__ << std::endl;

  //  m_matrix = new osg::RefMatrix;
}

SensorLink::SensorLink() : m_sensor(0), m_enable(true)
{
  // m_matrix = new osg::RefMatrix;
}

void SensorLink::setPropagate(OsgSensor::EnableFlag flag)
{
  if (!m_sensor) {
    osg::notify(osg::FATAL) << "SensorLink::setPropagate: sensor should not be null" << __FILE__ <<":" <<  __LINE__ << std::endl;
    return;
  }

  // Tell the sensor what to propagate to its matrix() method
  m_sensor->setPropagate(flag);
}

bool SensorLink::handle(int key)
{ 
  bool handled = false;
  if (m_sensor.valid() && m_enable) 
    handled = m_sensor->handle(key); 

  // If this SensorLink is dependent of another SensorLinks,
  // let them get this keystroke too
  if (m_dependents.size()) {
    DependentIterator di;
    for(di=m_dependents.begin(); di != m_dependents.end(); di++)
      handled = (*di)->handle(key)|| handled;
  }
  return handled;
}

void SensorLink::addDependent(SensorLink *link)
{
  if (!link) {
    osg::notify(osg::FATAL) << "SensorLink::addDependent(): link should not be null" << __FILE__ <<":" <<  __LINE__ << std::endl;
    return;
  }

  // We cant be dependent on our self, that would cause recursion
  if (link != this)
    m_dependents.push_back( link );
  else
    osg::notify(osg::FATAL) << "SensorLink::addDependent():  A Sensor link can't depend on it self" << std::endl;
}

bool SensorLink::removeDependent(SensorLink *link)
{
  if (!link)
    return false;

  if (m_dependents.size()) {
    DependentIterator di;
    for(di=m_dependents.begin(); di != m_dependents.end(); di++)
      if ((*di).get() == link) {
        m_dependents.erase(di);
        return true;
      }

  }
  return false;
}


osg::Matrix SensorLink::getMatrix() 
{
  if (!m_sensor) {
    osg::notify(osg::FATAL) << "SensorLink::getMatrix(): sensor not set." << __FILE__ <<":" <<  __LINE__ << std::endl;
    return osg::Matrix();
  }

  // Get the latest read value from the sensor
  if (m_enable) 
    m_matrix.set( m_sensor->getMatrix() );

  osg::Matrix m = m_matrix;
  // Should this matrix be inverted before multiplicated?
  if (m_sensor->getMatrixPropagateFlag() == OsgSensor::MatrixInvert) 
    m.invert(m);


  // If this SensorLink is dependent of another SensorLinks, postmultiply with those SensorLinks matrices
  if (m_dependents.size()) {
    DependentIterator di;
    for(di=m_dependents.begin(); di != m_dependents.end(); di++)
      m.postMult((*di)->getMatrix());
  }

  return m;
}

SensorLink::~SensorLink()
{
  m_dependents.clear();
}
