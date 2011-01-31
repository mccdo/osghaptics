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


#ifndef __HapticMaterial_h__
#define __HapticMaterial_h__

#include <osgHaptics/types.h>
#include <osgHaptics/export.h>

#include <map>

#include "HL/hl.h"
#include <osg/Referenced>
#include <osg/ref_ptr>
#include <osg/StateAttribute>



  namespace osgHaptics {
  
  /** Encapsulates HL  state. */
class OSGHAPTICS_EXPORT Material : public osg::StateAttribute
 {
  public :

    Material();

    inline void setStiffness(float s) {m_stiffness = s; }
    inline void setDamping(float d) { m_damping = d; }
    inline void setStaticFriction(float f) { m_static_friction = f; }
    inline void setDynamicFriction(float f) { m_dynamic_friction = f; }
    inline float getStiffness( ) const { return m_stiffness; }
    inline float getDamping( ) const {   return m_damping; }
    inline float getStaticFriction( ) const  { return m_static_friction; }
    inline float getDynamicFriction( ) const  { return m_dynamic_friction; }
    inline void setSide(HLenum side) { m_side = side; }

    /** Copy constructor using CopyOp to manage deep vs shallow copy. */
    Material(const Material& trans,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
      StateAttribute(trans,copyop),
      m_stiffness(trans.m_stiffness),
      m_damping(trans.m_damping),
      m_static_friction(trans.m_static_friction),
      m_dynamic_friction(trans.m_dynamic_friction),
      m_side(trans.m_side)
      {}

    META_StateAttribute(osg, Material,osg::StateAttribute::Type(OSGHAPTICS_MATERIAL))

      /** Return -1 if *this < *rhs, 0 if *this==*rhs, 1 if *this>*rhs. */
      virtual int compare(const osg::StateAttribute& sa) const
      {
        // Check for equal types, then create the rhs variable
        // used by the COMPARE_StateAttribute_Paramter macros below.
        COMPARE_StateAttribute_Types(Material,sa)

          // Compare each parameter in turn against the rhs.
        COMPARE_StateAttribute_Parameter(m_stiffness)
        COMPARE_StateAttribute_Parameter(m_static_friction)
        COMPARE_StateAttribute_Parameter(m_dynamic_friction)
        COMPARE_StateAttribute_Parameter(m_damping)
        COMPARE_StateAttribute_Parameter(m_side)

        return 0; // Passed all the above comparison macros, so must be equal.
      }

      virtual bool getModeUsage(ModeUsage& usage) const
      {
        return false;
      }

      virtual void apply(osg::State& state) const;


  protected :

    virtual ~Material() {};

    float m_stiffness;
    float m_damping;
    float m_static_friction;
    float m_dynamic_friction;
    HLenum m_side;    
  };

} // osgHaptics


#endif
