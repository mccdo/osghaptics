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

#ifndef __Haptic_ForceEffect_h__
#define __Haptic_ForceEffect_h__

#include <osg/Referenced>
#include <osg/Vec3d>


#include <HL/hl.h>
#include <map>

#include <osgHaptics/export.h>


  namespace osgHaptics {

    class HapticDevice;

    class OSGHAPTICS_EXPORT ForceEffect : public osg::Referenced 
    {
    public: 

      enum Type {
        CALLBACK_EFFECT,
        CONSTANT_EFFECT,
        SPRING_EFFECT,
        VISCOUS_EFFECT,
        FRICTION_EFFECT
      };

      HLenum getHLType() const;

      ForceEffect(HapticDevice *device, Type=CONSTANT_EFFECT);      

      /// Modify the type of the effect
      void setType(Type);

      /// Return the type of the effect
      Type getType() const;

      /// Start the effect, run until stop is called
      virtual void start();
      
      /// Update the effect with new parameters set
      virtual void update();

      /// Stop the effect
      virtual void stop();

      /// Start the effect and run in duration ms.
      virtual void trig();

      // Mutators
      void setGain(HLdouble gain);
      void setMagnitude(HLdouble magnitude);
      void setFrequency(HLdouble f);
      void setDuration(HLdouble milliseconds);
      void setPosition(const osg::Vec3d& pos);
      void setDirection(const osg::Vec3d& dir);

      // Accessors
      HLdouble getGain() const;
      HLdouble getMagnitude() const;
      HLdouble getFrequency() const;
      HLdouble getDuration() const;
      osg::Vec3d getPosition() const;
      osg::Vec3d getDirection() const;
	  bool getStarted() const { return m_started; }

    protected:
      virtual ~ForceEffect();

      /*! 
        Callback for CALLBACK type of ForceEffect.         
        This callback will be executed in the haptic servo loop
        Called upon START of a Force effect
      */      
      virtual void startCB() {}

      /*! 
      Callback for CALLBACK type of ForceEffect.         
      This callback will be executed in the haptic servo loop
      */      
      virtual void computeCB(const osg::Vec3d& in, osg::Vec3d& out) { out = in; }

      /*! 
      Callback for CALLBACK type of ForceEffect.         
      This callback will be executed in the haptic servo loop
      Called upon STOP of a Force effect
      */      
      virtual void stopCB() {}

      friend class HapticDevice;
      enum Operation {
        START,
        STOP,
        TRIG,
        UPDATE
      };

      void executeStart();
      void executeStop();
      void executeTrig();
      void executeUpdate();

      void applyParameterset() const;

	  bool m_started;

      HLuint m_effect_id;
      typedef std::map<HLenum, HLfloat> FloatMap;
      FloatMap m_float_parameters;
      
      typedef std::map<HLenum, osg::Vec3d> Vec3Map;
      Vec3Map m_vec3_parameters;


      // A weak reference to the HapticDevice
      HapticDevice *m_device;

      Type m_type; 
    };
} // namespace osgHaptics

#endif
