/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Sensor Library
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

#ifndef __SensorEventHandler_h__
#define __SensorEventHandler_h__

#include <osg/Referenced>
#include <queue>
#include <osgSensor/export.h>
#include <bitset>
namespace osgSensor {

  class Sensor;


  /// Callback class for handling events
  /*!
    This class can store the values of different sensor button and or Valuators.
    A Button can have two states UP/DOWN
    A valuator is defined to have a float value in the interval of [0..1]
    An example of a Sensor with a valuator is Joystick (Gamepad) where each axis will defined
    a value in the interval 0..1 no matter what the original defined value interval was.

    There are various types of events, each time the Sensor is updated a UPDATE event is generated,
    when a button is pressed a BUTTON event is generated.

    For each event the ()operator is called with the appropriate values set.

    Which Events that will trigger the ()operator can be set using the activation mask.
    
    When it comes to Button events there are two alternatives.
    Either an event is generated for each button press, or the button presses can be OR:ed into a button-mask 
    which can then be queried in one single BUTTON event. This can be specified with setEventMode()
  */
  class OSGSENSOR_EXPORT SensorEventHandler : public osg::Referenced {
  public:


    /*!
      Default constructor
      \param name - Name of this SensorEventHandler
     */
    SensorEventHandler(const std::string& name="");

    /// Return the name of this SensorEventHandler
    std::string getName() const { return m_name; }

    /// Defines the buttons that can be used for a general Sensor device
    enum Button { 
      BUTTON_NONE,
      BUTTON_1, 
      BUTTON_2,
      BUTTON_3,
      BUTTON_4,
      BUTTON_5,
      BUTTON_6,
      BUTTON_7,
      BUTTON_8,
      BUTTON_9,
      BUTTON_10,
      BUTTON_11,
      BUTTON_12,
      BUTTON_13,
      BUTTON_14,
      BUTTON_15,
      MAX_NUM_BUTTONS 
    };


    /// Defines the Valuators that can be used for a general Sensor device
    enum Valuator { 
      VALUATOR_NONE,
      VALUATOR_1, 
      VALUATOR_2,
      VALUATOR_3,
      VALUATOR_4,
      VALUATOR_5,
      VALUATOR_6,
      VALUATOR_7,
      VALUATOR_8,
      VALUATOR_9,
      VALUATOR_10,
      VALUATOR_11,
      VALUATOR_12,
      VALUATOR_13,
      VALUATOR_14,
      VALUATOR_15,
      MAX_NUM_VALUATORS 
    };

    /// The available event modes
    enum EventMode {
      IMMEDIATE,  /// For each BUTTON event, execute a callback
      QUEUE_EVENTS /// Queue up the BUTTON events and OR the buttonMask together and execute ONE BUTTON event.
    };

    /// Specify the eventmode. See EventMode
    void setEventMode(EventMode eventMode) { m_eventMode = eventMode; }

    /// Return the current eventmode
    EventMode getEventMode() const { return m_eventMode; }

    /// Specify the available types of Events that can occur
    enum EventType {
      TYPE_NONE, /// Undefined event-type
      BUTTON,    /// A Button press/release event
      UPDATE,    /// Sensor::update() is called
      SHUTDOWN,  /// The Sensor::shutdown() is called
      INITIALIZE, /// Sensor::init() is called
      DISCONNECT, /// The sensor is disconnected
      CONNECT,    /// THe sensor is re-connected
      LAST_EVENT_TYPE=CONNECT,
      ALL=BUTTON|UPDATE|SHUTDOWN|INITIALIZE|DISCONNECT|CONNECT
      //VALUATOR?
    };

    /// Defines the state of the button, STATE_NONE is for uninitialized values
    enum ButtonState {
      STATE_NONE,
      DOWN,
      UP
    };

    /*!
    Specify which events that should trigger the () operator to be executed.
    For example: setActivationMask(UPDATE|BUTTON).
    Default is ALL. 
    */
    void setActivationMask(unsigned int mask);

    /*!
    Set the specified eventType to be enabled/disabled
    */
    void setActivationBit(EventType eventType, bool enabled_flag=true);

    /// Return true if eventType is enabled for activation
    bool getActivationBit(EventType eventType)const;



    /// Class to store an event
    struct Event {
      Event(float t, EventType type, Button b=BUTTON_NONE, ButtonState state=STATE_NONE) : 
        time(t), eventType(type), button(b), buttonState(state) {}

      float time;
      EventType eventType;
      Button button;
      ButtonState buttonState;
    };

    typedef std::vector<float> ValuatorVector;
    typedef ValuatorVector::iterator ValuatorVectorIterator;    
    typedef std::vector<ButtonState> ButtonStateVector;
    
    /// Return the vector of button states (UP/DOWN)
    const ButtonStateVector& getButtonStateVector() const { return m_buttonState; }

    /// Return the vector of valuator values [0..1]
    const ValuatorVector &getValuatorVector() const { return m_valuatorValues; }


    /// Return the number of buttons that the associated Sensor device has available
    unsigned int getNumButtons() const { return m_numberOfButtons; }

    /// Return the number of valuators that the associated Sensor device has available
    unsigned int getNumValuators() const { return m_numberOfValuators; }

    ///TODO: Add functionality for queryButton
    /// Returns true if button b is available
    bool queryButton(Button b);

    ///TODO: Add functionality for queryValuator
    /// Returns true if valuator v is available
    bool queryValuator(Valuator v);

    /// Return the current value of the valuator
    float getValuatorValue(Valuator v) const;

    /// \return the button pressed during the current event, if no button was pressed BUTTON_NONE is returned
    Button getButton() const {return m_button; }

    /// \return the event type that dispatched the event callback
    EventType getEventType() { return m_eventType; }



    /*!  
      The callback method which will be dispatched for each event.
      \param time - The current time of the event
     */
    virtual void operator() (EventType eventType, float time) =0;

    /*! 
      Should be called BEFORE pushEvent to mark the start of a new event
      \param sensor - The sensor that initiates the begin call, just send 'this'
    */
    void begin(osgSensor::Sensor *sensor);

    /// Dispatch a new event
    void pushEvent(float t, unsigned int type=UPDATE, Button b=BUTTON_NONE, ButtonState state=STATE_NONE)
    {
      pushEvent(t,(EventType)type,b,state);
    }


    /// Dispatch a new event
    void pushEvent(float t, EventType type=UPDATE, Button b=BUTTON_NONE, ButtonState state=STATE_NONE);

    /// Dispatch a new event
    void pushEvent(Event& e) { pushEvent(e.time, e.eventType, e.button, e.buttonState); }

    /// Must be called after a call to begin()..pushEvent() in QUEUE_EVENTS it will actually dispatch the queued events
    void end();

    /// Set the state of a button
    void setButtonState(Button b, ButtonState state);

    /*! 
      Return the state of a specified button
    */
    ButtonState getButtonState(Button b=BUTTON_NONE) const ;

    /// Set the value of a specified Valuator
    void setValuatorValue(Valuator v, float value);

    /// Set the type of the current event
    void setEventType(EventType type) { m_eventType = type; }

    /// Set the number of buttons that the accociated Sensor device is capable of
    void setNumberOfButtons(unsigned int n) { m_numberOfButtons = n;  m_buttonState.resize(n+1); }

    /*! 
      Set the number of Valuators that the accociated Sensor device is capable of
      For example a GamePad commonly has at least 3 (x,y&z) and usually one more
     */
    void setNumberOfValuators(unsigned int n) { m_numberOfValuators = n; m_valuatorValues.resize(n+1); }

    osgSensor::Sensor *getSensor() const { return m_sensor; }

  protected:

    /// Destructor
    virtual ~SensorEventHandler();

    typedef std::queue<Event> EventQueue;
    EventQueue m_eventQueue;

    bool m_initialized;
    unsigned int m_buttonMask;
    bool m_useIndividualEvents;
    ButtonStateVector m_buttonState;
    EventType m_eventType;
    unsigned int m_numberOfButtons, m_numberOfValuators;
    std::string m_name;
    bool m_beginCalled;
    Button m_button;
    EventMode  m_eventMode;
    ValuatorVector m_valuatorValues;

    // store the activationmask as bits, make sure there are plenty of room
    std::bitset<LAST_EVENT_TYPE*10> m_activationMask;

    Sensor *m_sensor;
  };

} // namespace sensors

#endif
