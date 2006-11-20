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

#include <osgHaptics/ShapeComposite.h>

using namespace osgHaptics;

ShapeComposite::~ShapeComposite()
{
}


void ShapeComposite::setEnable(bool flag) 
{ 
  Shape::setEnable(flag);

  ShapeMap::iterator it=m_children.begin();

  for(; it != m_children.end(); it++)
    it->second->setEnable(flag);
}


bool ShapeComposite::contains(const Shape *shape) const
{
  ShapeMap::const_iterator it=m_children.find(shape);
  if (it != m_children.end()) 
    return true;

  return false;
}

bool ShapeComposite::contains(HLuint shape_id) const
{
  ShapeIDMap::const_iterator it=m_children_id.find(shape_id);
  if (it != m_children_id.end()) 
    return true;

  return false;
}
