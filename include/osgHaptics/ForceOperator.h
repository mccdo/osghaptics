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

#ifndef __ForceOperator_h__
#define __ForceOperator_h__

#include <osg/Referenced>
#include <OpenThreads/Mutex>
#include <osg/Vec3d>
#include <osg/Timer>
#include <osg/Matrix>

#include <osgHaptics/export.h>



  namespace osgHaptics {
  
    class HapticDevice;

    class OSGHAPTICS_EXPORT ForceOperator : public osg::Referenced {
    public:
      ForceOperator() : m_enabled(true), m_trigged(false), m_duration(0), m_start(0) {}

      friend class HapticDevice;
      /// Calculate the force this Operator should affect the haptic device
      virtual void calculateForce(const osg::Vec3d& in, osg::Vec3d& out, double time) { out = in; }

      /// Calculate the Torque this Operator should affect the haptic device
      virtual void calculateTorque(const osg::Vec3d& in, osg::Vec3d& out, double time) { out = in; }

      /// Enable/disable the effect
      virtual void setEnable(bool f) { OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex); m_enabled = f; }

      /// Return wether the force effect is enabled or not
      bool getEnable() const { OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex); bool f = m_enabled; return f; }
    
      /// Enable this effect for a specified time
      void trig(unsigned int milliseconds_duration);

      void setWorldToWorkSpaceMatrix(const osg::Matrix& m);
      void getWorldToWorkSpaceMatrix(osg::Matrix& m) const;
      void getWorkspaceToWorldMatrix(osg::Matrix& m) const;

    protected:
      void update();

      mutable OpenThreads::Mutex m_mutex;
      virtual ~ForceOperator() {}
      osg::Matrix m_world_to_workspace_matrix, m_workspace_to_world_matrix;

    private:
      bool m_trigged;
      osg::Timer_t m_start;
      double m_duration;
      bool m_enabled;
    };
  } // namespace osgHaptics

#endif
