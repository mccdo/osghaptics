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

#ifndef __osgHaptics_EventHandler_h__
#define __osgHaptics_EventHandler_h__

#include <osg/Referenced>



  namespace osgHaptics {

/// Callback class for handling events
class EventHandler : public osg::Referenced {

public:
  friend class HapticDevice;

  enum Button { 
    BUTTON_1, 
    BUTTON_2, 
    NUM_BUTTONS 
  };


  enum EventType {
    BUTTON,
    CALIBRATION
  };

  enum ButtonState {
    DOWN,
    UP
  };

  enum CalibrationType {
    UPDATE,
    INPUT
  };

  EventHandler() { m_button_state[BUTTON_1] = UP; m_button_state[BUTTON_2] = UP; }

  /// Return the ButtonState of a specified button
  ButtonState getState(Button b) const { return m_button_state[b]; }

  EventType getEventType() { return m_event_type; }

  /// Virtual method that will be called upon a button press
  virtual void push(Button b) { }

  /// Virtual method that will be called upon a button release
  virtual void release(Button b) { }

  /// Virtual method that will be called upon a calibration event
  virtual void calibrate(CalibrationType type) {  }

protected:
  /// Set the state of a button
  void setState(Button b, ButtonState state) { m_button_state[b] = state; }

  void setType(EventType type) { m_event_type = type; }

  friend class HapticDevice;
  virtual ~EventHandler() {}
  EventType m_event_type;
  ButtonState m_button_state[2];
};

} // namespace osgHaptics

#endif
