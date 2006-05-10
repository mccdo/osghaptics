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
#include <osgSensor/OsgSensorCallback.h>
#include <osg/MatrixTransform>
#include <osg/Notify>

using namespace osgSensor;

OsgSensorCallback::OsgSensorCallback(OsgSensor *sensor) : m_sensor(sensor), m_previousTraversalNumber(-1), m_enable(true)
{
  if (!m_sensor)
    osg::notify(osg::FATAL) << "OsgSensorCallback: Null pointer as sensor is invalid: " << __FILE__ << ": " << __LINE__ << std::endl;
}

void  OsgSensorCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{ 
  if (m_enable && nv->getTraversalNumber()!=m_previousTraversalNumber) {
    m_previousTraversalNumber = nv->getTraversalNumber();
    update(node);
  }
  traverse(node,nv);
}

void OsgSensorCallback::update(osg::Node *node)
{
  osg::MatrixTransform *t = dynamic_cast<osg::MatrixTransform *> (node);
  if (t) {
    // Update the node with the matrix of the sensor
    t->setMatrix(m_sensor->getMatrix());
  }
  else
    osg::notify(osg::FATAL) << "OsgSensorCallback: Node should be a MatrixTransform: " << __FILE__ << ": " << __LINE__ << std::endl;

}
