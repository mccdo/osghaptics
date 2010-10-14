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

#include <osg/MatrixTransform>
#include <osgSensor/SensorLinkCallback.h>
#include <iostream>

using namespace osgSensor;


SensorLinkCallback::SensorLinkCallback( SensorLink *link) : m_link(link), m_previousTraversalNumber(-1), m_enabled(true)
{
  if (!m_link)
    osg::notify(osg::WARN) << "SensorLinkCallback::operator(): link should not be null!" << __FILE__ <<":" << __LINE__ << std::endl;
}

void SensorLinkCallback::operator()(osg::Node* node, osg::NodeVisitor* nv)
{ 
  if (m_enabled && nv->getTraversalNumber()!=m_previousTraversalNumber) {
    m_previousTraversalNumber = nv->getTraversalNumber();
    update(node);
  }
  traverse(node,nv);
}


void SensorLinkCallback::update(osg::Node *node) {
  osg::MatrixTransform *t = dynamic_cast<osg::MatrixTransform *> (node);
  if (!t) {
    osg::notify(osg::WARN) << "SensorLinkCallback::update(): Node should be a Transform *, but is not!" << __FILE__ <<":" <<  __LINE__ << std::endl;
    return;
  }

  // Update the node with the matrix of the sensor
  t->setMatrix(m_link->getMatrix());
}

