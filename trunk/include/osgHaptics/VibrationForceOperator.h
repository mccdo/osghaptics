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

#ifndef __VibrationForceOperator_h__
#define __VibrationForceOperator_h__

#include <osgHaptics/ForceOperator.h>
#include <osgHaptics/export.h>



namespace osgHaptics {

    class OSGHAPTICS_EXPORT VibrationForceOperator : public ForceOperator {
    public:
      
      /// Default constructor
      VibrationForceOperator();

      void setDirection( osg::Vec3d& direction );
      osg::Vec3d getDirection() const { return m_direction; }

      void setFrequency( float f );
      double getFrequency() const { return m_frequency; }

      void setAmplitude( double amplitude );
      double getAmplitude() const { return m_amplitude; }

      virtual void calculateForce( const osg::Vec3d& in, osg::Vec3d& out, double time );
      virtual void calculateTorque( const osg::Vec3d& in, osg::Vec3d& out, double time ) { out = in; }

    protected:
      
      /// Destructor
      virtual ~VibrationForceOperator() {}

    private:
      double m_amplitude;
      double m_max_amplitude;
      float m_frequency;
      osg::Vec3d m_direction;
    };

  } // namespace osgHaptics




#endif
