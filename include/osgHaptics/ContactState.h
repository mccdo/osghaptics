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

#ifndef __OsgHaptics_ContactState_h__
#define __OsgHaptics_ContactState_h__




#include <osg/Vec3>
#include <osg/Referenced>
#include <osgHaptics/export.h>
#include <iostream>
#include <osg/io_utils>




  namespace osgHaptics {
  
  class Shape;
  class HapticDevice;
    /// ContactState contains information about a contact event between two objects or two materials

  /*
  This class stores the information generated during a contact, separation or continous contact between 
  two objects, or two materials. It always contain the pair of objects involved in the contact,
  as well as the pair of materials involved.
  The other fields, such as normals, torque etc is only valid for contact. (Not during separation as
  there is no forces or normals involved).
  */
  class OSGHAPTICS_EXPORT ContactState : public osg::Referenced
  {
  public:

    OSGHAPTICS_EXPORT friend std::ostream& operator<<(std::ostream& os, ContactState& state);

    /// Indicates which type of event that has occured
    enum ContactEvent { 
      None=0x0,        /// No event at all
      Separation=0x1, /// Proxy has separated from shape
      Contact=0x2,    /// Proxy has first contact
      Motion=0x4,  /// Contact and moving
      All=Separation|Contact|Motion // All of the above
    };


  public:


    /// Base constructor
    ContactState() { clear(); }

    ContactState(ContactEvent event, Shape *shape, HapticDevice *device, double time) : Referenced()
    {
      //ContactState();
      set(event, shape, device, time);
    }

    /// Copy constructor
    ContactState(const ContactState& o) : Referenced(o) {
      if (this == &o)
        return;
      *this = o;
    }

    /// Copy operator
    ContactState& operator =(const ContactState& o) {
      if (this == &o)
        return *this;

      m_event = o.m_event;
      m_shape = o.m_shape;
      m_time = o.m_time;

      m_position = o.m_position;
      m_normal = o.m_normal;
      m_force = o.m_force;
      m_torque = o. m_torque;


      m_velocity = o.m_velocity;

      return *this;
    }

    /// Destructor
    virtual ~ContactState() {}

    /// Resets the state to its default values
    void clear() {
      m_time = 0;
      m_event = None;
      m_position.set(0,0,0);
      m_normal.set(0,0,0);
      m_force.set(0,0,0);
      m_torque.set(0,0,0);

      m_shape=0L; 
      m_velocity.set(0,0,0);
    }

    /// Sets the state of the collision event this method is called by the collision manager
    void set(ContactEvent event, Shape* shape, HapticDevice *, double time);

    /// Sets the state of the collision event this method is called by the collision manager
    void set(ContactEvent event, Shape* shape, HapticDevice *,const osg::Vec3& pos, const osg::Vec3& normal, double time);

    /// Return one of the objects involved in the collision
    Shape  *getShape()  { return m_shape; }


    /// \return the position of the contact
    osg::Vec3 getContactPosition ()  const { return m_position; }

    /// \return the normal of the first contact
    osg::Vec3 getContactNormal ()  const { return m_normal; }

    /// \return the velocity of the proxy at contact time
    osg::Vec3 getVelocity ()  const { return m_velocity; }

    /// Return time of event (in simulation time)
    double getTime() const { return m_time; }

    /// Return the type of the event that has occured
    ContactEvent getEvent() const { return m_event; }


  protected:
    virtual const char *className() { return "ContactState"; }
    std::string classType() { return "ContactState"; }

  private:


    ContactEvent m_event;
    osg::Vec3 m_position;
    osg::Vec3 m_normal;
    osg::Vec3 m_force;
    osg::Vec3 m_torque;
    double m_time;

    //osg::ref_ptr<Shape> m_shape; 
    Shape  *m_shape; 
    osg::Vec3 m_velocity;
  };

  


} // Namespace osgHaptics


#endif

