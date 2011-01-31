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

#ifndef __SpringForceOperator_h__
#define __SpringForceOperator_h__

#include <osgHaptics/ForceOperator.h>
#include <osgHaptics/export.h>
#include <osg/Vec3>



  namespace osgHaptics {

    class OSGHAPTICS_EXPORT SpringForceOperator : public ForceOperator {
    public:
      SpringForceOperator();

      /*!
        Set the anchor position of the spring.
        \param position - Position of the anchor
        \param fadein_ms - The time to interpolate from current position to the specified one.
        If fadein_ms == 0, then jump to the new position immediately.
      */
      void setPosition( const osg::Vec3d& position, double fadein_ms=0 );
      void setPosition( const osg::Vec3& position, double fadein_ms=0 ) { setPosition(osg::Vec3d(position), fadein_ms); }

      /*!
        Toggle the SpringForce
        \param flag - If true then enable the SpringForce immediately, else disable
      */
      virtual void setEnable(bool flag);

      /*!
        Toggle the spring force.
        This method can be used to fade in/fade out a spring.
        The second parameters specifies the fade in/out time.
        The spring wont actually be disabled in the call to this method, it will in stead
        be automatically disabled when fade_ms seconds has elapsed

        \param flag - If true then this SpringForce will be enabled gradually.
        \param fade_ms - Number of ms to gradually increase/decrease the spring force
      */
      virtual void setEnable(bool flag, double fade_ms);

      /// Return the current position of the anchor (the interpolated one)
      osg::Vec3d getPosition() const {   
        OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
        osg::Vec3d pos = m_current_position;
        return pos; 
      }

      void setStiffness( double stiffness );
      double getStiffness() const {   
        OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
        double s = m_stiffness;
        return s; 
      }

      void setDamping( double damping );
      double getDamping() const {   
        OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_mutex);
        double s = m_damping;
        return s; 
      }

      virtual void calculateForce( const osg::Vec3d& in, osg::Vec3d& out, double time );
      virtual void calculateTorque( const osg::Vec3d& in, osg::Vec3d& out, double time ) { out = in; }

    protected:

      /// Destructor
      virtual ~SpringForceOperator() {}

    private:
      double m_stiffness;
      double m_damping;
      double m_max_stiffness;
      osg::Vec3d m_position, m_target_position, m_current_position;
      bool m_fade;
      bool m_fade_started;
      double m_fadein_ms, m_fade_start_time;

      double m_current_fade;
      bool m_fade_force, m_fade_force_started;
      double m_fade_force_start_time, m_fade_force_time;
      bool m_fade_force_up;
    };

  } // namespace osgHaptics




#endif
