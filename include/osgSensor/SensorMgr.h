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

#ifndef __SensorManager_h__
#define __SensorManager_h__
#include <cassert>

#include <osg/Referenced>
#include <osgSensor/export.h>
#include <map>
#include <osgSensor/Sensor.h>

namespace sensors {
  class Sensor;
  
}

namespace osgSensor {

  /// SensorMgr is a class for handling all sensor devices.

  /*!
    All classes inherited from the Sensor class will automatically be registrated to this
    manager.

    When SensorMgr::update() is called, all registrated Sensors update method will be called.

    Same goes for SensorMgr::shutdown()

  */ 
  class OSGSENSOR_EXPORT SensorMgr : public osg::Referenced
  {
  public:


    /// Return the singleton object
    static SensorMgr *instance( void );

    /*!
      Shutdown all registrated sensors
    */
    void shutdown();

    /*!
    Initializes the SensorManager.
    */
    bool init();

    /// Call update on all registrated sensors
    void update();

    /// Registrate a sensor
    void registerSensor(osgSensor::Sensor *sensor);

    /// unregistrate a sensor.
    bool unRegisterSensor(osgSensor::Sensor *sensor);

    /*! Find a named sensor and return it.
      \returns return a pointer to the named sensor, NULL if no sensor is found
    */
    Sensor *find(const std::string& name);

  private:

    typedef std::multimap< std::string, osg::ref_ptr<osgSensor::Sensor>  > SensorMap;
    SensorMap m_sensors;

    /// Destructor
    virtual ~SensorMgr();

    /// Constructor
    SensorMgr( void );

    bool m_shutdown;
  };


#define g_SensorMgr osgSensor::SensorMgr::instance()


} // namespace sensors


#endif

