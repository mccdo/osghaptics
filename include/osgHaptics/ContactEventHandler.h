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

#ifndef __ContactEventHandler_h__
#define __ContactEventHandler_h__



#include <osg/Referenced>

#include <osgHaptics/export.h>
#include <osgHaptics/ContactState.h>




  namespace osgHaptics {
  


  /// Class for acting upon a contact event.

  /*!
    This class is a base class for handling events during contact.
    An object of this class can be registrated for specific contact events.
    When that contact occurs between the proxy and the registrated shape, 
    the virtual methods will be called, with the current ContactState
    as a parameter.
  */
  class OSGHAPTICS_EXPORT ContactEventHandler : public osg::Referenced
  {
  public:

    friend class HapticDevice;

    /// Base constructor
    ContactEventHandler(){};

    /*! Virtual method called when a contact between proxy and the shape has occured.
    \param state - Contains the collision information for the current collision.
    */
    virtual void contact( ContactState& state){};

    /*! Virtual method called when proxy and the shape previously in contact separates
    \param state - Contains the collision information for the current collision.
    
    */
    virtual void separation( ContactState& state){};

    /*! Virtual method called when proxy is in motion over the shape, but still in contact
    \param state - Contains the collision information for the current collision. 
    */
    virtual void motion( ContactState& state){};

  protected:

    /// Executes the appropriate virtual method depending on the state
    void execute( ContactState& state);

  protected:

    /// Destructor
    virtual ~ContactEventHandler() {}

  private:
  };


  inline void ContactEventHandler::execute( ContactState& state) {

    switch(state.getEvent()) {

      /// Separation between pair, execute separation() method
    case (ContactState::Separation):
      separation(state);
      break;

      /// A new contact, execute contact() method
    case (ContactState::Contact):
      contact(state);
      break;

      /// Continous contact, execute continous() method
    case (ContactState::Motion):
      motion(state);
      break;
    }
  }

} // namespace osgHaptics

#endif
