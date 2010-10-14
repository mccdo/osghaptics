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

#include <osgHaptics/HapticSpringNode.h>

using namespace osgHaptics;

HapticSpringNode::HapticSpringNode(HapticDevice *device) : Transform(), m_device(device)

{

  // Dont cull nor optimize this group away.
  setCullingActive(false);
  setDataVariance(osg::Object::DYNAMIC);

  m_force_operator = new osgHaptics::SpringForceOperator();
  m_device->addForceOperator(m_force_operator.get());
}


void HapticSpringNode::traverse(osg::NodeVisitor &nv)
{
  if (!m_force_operator.valid())
    return;

  if (!nv.getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR) 
    return;

  osg::Matrix m = computeLocalToWorld(nv.getNodePath());
  osg::Vec3 pos = m.getTrans();

  m_force_operator->setPosition(m.getTrans());
}

HapticSpringNode & HapticSpringNode::operator=(const HapticSpringNode &node)
{ 
  if (this == &node) return *this; 


  return *this;
}


HapticSpringNode::HapticSpringNode(const HapticSpringNode &copy, const osg::CopyOp &copyop)
: osg::Transform(copy, copyop)
{
  *this = copy;
}


HapticSpringNode::~HapticSpringNode()
{
  if (m_force_operator.valid()) {
    m_force_operator->setEnable(false);
    m_device->removeForceOperator(m_force_operator.get());
  }
}

