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

#ifndef __HapticTouchModel_h__
#define __HapticTouchModel_h__

#include <HL/hl.h>

#include <osg/StateAttribute>
#include <osg/Referenced>
#include <osg/ref_ptr>

#include <osgHaptics/types.h>
#include <osgHaptics/export.h>

#include <map>


  namespace osgHaptics {

    /** Encapsulates HL  state. */
    class OSGHAPTICS_EXPORT TouchModel : public osg::StateAttribute
    {
    public:

      enum Mode {
        CONTACT = 0,
        CONSTRAINT = 1
      };

      TouchModel(Mode m=CONTACT);

      void setMode(Mode m) { m_mode = m; }
      Mode getMode() const { return m_mode; }

      void setSnapDistance(HLdouble distance) { m_snap_distance = distance; }
      HLdouble getSnapDistance() const { return m_snap_distance; }

      /*
      hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);

      // We can get a good approximation of the snap distance to use by
      // solving the following simple force formula: 
      // >  F = k * x  <
      // F: Force in Newtons (N).
      // k: Stiffness control coefficient (N/mm).
      // x: Displacement (i.e. snap distance).

      */
      static float calcForceToSnapDistance(float force);

      /** Copy constructor using CopyOp to manage deep vs shallow copy. */
      TouchModel(const TouchModel& trans,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
      StateAttribute(trans,copyop),
        m_mode(trans.m_mode),
        m_snap_distance(trans.m_snap_distance)
      {}

      META_StateAttribute(osg, TouchModel,osg::StateAttribute::Type(OSGHAPTICS_TOUCH_MODEL));

      /** Return -1 if *this < *rhs, 0 if *this==*rhs, 1 if *this>*rhs. */
      virtual int compare(const osg::StateAttribute& sa) const
      {
        // Check for equal types, then create the rhs variable
        // used by the COMPARE_StateAttribute_Paramter macros below.
        COMPARE_StateAttribute_Types(TouchModel,sa)

        // Compare each parameter in turn against the rhs.
        COMPARE_StateAttribute_Parameter(m_mode)
        return 0; // Passed all the above comparison macros, so must be equal.
      }

      virtual bool getModeUsage(ModeUsage& usage) const
      {
        return false;
      }

      virtual void apply(osg::State& state) const;

    protected:
      
      /// Destructor
      virtual ~TouchModel() {}

    private:
      HLdouble m_snap_distance;
      Mode m_mode;

    };

  } // osgHaptics


#endif
