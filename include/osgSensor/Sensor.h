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

#ifndef __Sensor_h__
#define __Sensor_h__

#include <osg/Vec3>
#include <osg/Quat>
#include <osg/Matrix>
#include <osg/Referenced>

#include <vector>
#include <cassert>
#include <osgSensor/export.h>
#include <osgSensor/SensorEventHandler.h>
#include <map>
/**
* \mainpage
* osgSensor is a set of libraries/classes for using sensor devices such as 6DOF tracking systems in
the OpenSceneGraph rendering library.
*/

namespace osgSensor {
/// Abstract base class for all sensor devices.

/*!
  6DOF and other devices can inherit from this class.
  update(), read(), getNumberOfSensors and shutdown() are methods that has to be implemented
*/
  class OSGSENSOR_EXPORT Sensor : public osg::Referenced {
  
  public:

    Sensor();
    Sensor(const std::string& name);

  /// Returns the current sensor information for sensor_no. 
  virtual int read(unsigned int sensor_no, osg::Vec3& p, osg::Quat& q, unsigned long timeout=2000 ) { assert(0); return 0; }

  /// Returns the sensor information
  virtual int read(osg::Vec3& p, osg::Quat& q) { assert(0); return 0; } ;

  /// Read the first sensor (or the only) and set the transformation matrix
  virtual int read(osg::Matrix& matrix ) { assert(0); return 0; };

  /// Return the number of 6Dof sensors (if any)
  virtual unsigned int getNumberOfSensors()=0;

  /// Return the number of buttons that this Sensor has
  virtual unsigned int getNumberOfButtons() =0;

  /// Return the number of Valuators that this Sensor has
  virtual unsigned int getNumberOfValuators() =0;

  /// Update the internal state of the sensors. Usually means that data is read from the actual device
  virtual void update(float time=0.0f)=0;
  
  /// Tell the device to shut down everything.
  virtual void shutdown(float time=0.0f)=0;
  
  /// Set the timeout for reading from external device
  void setTimeout(unsigned int ms=5000) { m_timeout = ms; }

  /// REturn the timeout for reading from an external device
  unsigned int getTimeout(unsigned int ms=5000) const { return m_timeout; }

  virtual void dirty() {}
  virtual bool isDirty() { return false; }
  
  /// Set the name of this sensor
  void setName( const std::string& name ) { m_name = name; }

  /// Return the name of this sensor
  std::string getName(  ) const { return m_name; }


  /// Register a SensorEventhandler that will be dispatched for events
  void registerSensorEventHandler(osgSensor::SensorEventHandler *eventHandler);

  /// Register a SensorEventhandler that will be dispatched for events
  bool unregisterSensorEventHandler(osgSensor::SensorEventHandler *eventHandler);

  void execShutdown(float time=0);

  void execUpdate(float time=0);

  typedef std::map<SensorEventHandler *, osg::ref_ptr<SensorEventHandler> > EventHandlerMap;
  typedef EventHandlerMap::iterator EventMapIterator;

  EventHandlerMap &getEventHandlers() { return m_eventHandlers; }

  protected:

    /// Implement this method so that all event handlers are updated
    virtual void updateEventHandlers(float time); 

    void executeEvent(osgSensor::SensorEventHandler::Event &e);

    EventHandlerMap m_eventHandlers;

  std::string m_name;
  bool m_initialized;
  unsigned int m_timeout;
  bool m_shutdown;
  virtual ~Sensor();
};


class OSGSENSOR_EXPORT DummySensor : public Sensor {

public:

  DummySensor() : Sensor() {}

  /// Returns the current sensor information for sensor_no. 
  virtual int read(unsigned int sensor_no, osg::Vec3& p, osg::Quat& q, unsigned long timeout=2000 );

  /// Returns the sensor information
  virtual int read(osg::Vec3& p, osg::Quat& q);

  /// Read the first sensor (or the only) and set the transformation matrix
  virtual int read(osg::Matrix& matrix );

  /// Return the number of 6Dof sensors (if any)
  virtual unsigned int getNumberOfSensors() { return 1; }

  /// Return the number of valuators (if any)
  virtual unsigned int getNumberOfValuators() { return 1; }

  /// Return the number of buttons (if any)
  virtual unsigned int getNumberOfButtons() { return 1; }


  /// Set the timeout for reading from external device
  void setTimeout(unsigned int ms=5000) { m_timeout = ms; }

  /// REturn the timeout for reading from an external device
  unsigned int getTimeout(unsigned int ms=5000) const { return m_timeout; }


protected:

  /// Update the internal state of the sensors. Usually means that data is read from the actual device
  virtual void update(float time=0.0f) {}

  /// Tell the device to shut down everything.
  virtual void shutdown(float time=0.0f) {}


  unsigned int m_timeout;
  virtual ~DummySensor() {}
  };


} // namespace sensors

#endif
