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


#include <osgHaptics/Material.h>
#include <osg/Notify>



using namespace osgHaptics;

Material::Material() : StateAttribute(), 
  m_stiffness(0.7f), m_damping(0.1f), 
  m_static_friction(0.2f), 
  m_dynamic_friction(0.3f), 
  m_side(HL_FRONT_AND_BACK) 
{
}

void Material::apply(osg::State& state) const
{
  //hlPushAttrib(HL_MATERIAL_BIT);

  hlMaterialf(m_side, HL_STIFFNESS, m_stiffness);
  hlMaterialf(m_side, HL_DAMPING, m_damping);
  hlMaterialf(m_side, HL_STATIC_FRICTION, m_static_friction);
  hlMaterialf(m_side, HL_DYNAMIC_FRICTION, m_dynamic_friction);
}
