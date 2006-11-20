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
#ifndef __osgsensors_ThreadedSensor_h__
#define __osgsensors_ThreadedSensor_h__

#include <osgSensor/StopThread.h>
#include <osgSensor/Sensor.h>
#include <osgSensor/export.h>
#include <map>
#include <vector>
#include <osg/io_utils>



namespace osgSensor {

class  OSGSENSOR_EXPORT ThreadedSensor : public vrutils::StopThread, public Sensor {
public:  

  /// Constructor  
  ThreadedSensor( Sensor *sensor );


   /// Initialises the connection to the server
  bool init( unsigned int num_sensors );
    
  unsigned int getNumberOfSensors() { unsigned int i; OpenThreads::ScopedLock<OpenThreads::Mutex> ml(m_io_mutex); i = m_sensor->getNumberOfSensors();  return i; }
   
   /*! 
    Get the last dataset from the device for all sensors
    and print the data to the specified stream.
    If device doesnt respont within DEFAULT_TIMEOUT, its a failure
    \returns 1 upon success, 0 for failure
   */
  int read( std::ostream& ostr=std::cerr, unsigned long timeout=3000 );

   /*! 
      Get the last dataset from the device for the specified sensors
      and print the data to the specified stream.
      If device doesnt respont within DEFAULT_TIMEOUT, its a failure
      \returns 1 upon success, 0 for failure
   */
   int read(unsigned int sensor_no, osg::Vec3& p, osg::Quat& q, unsigned long timeout=3000 );

  
   /*! Shuts down the connection to the server. After this no other calls should be made
    to this api except the destructor
   */
   virtual void shutdown( void );

   /// Read data from the device and update the internal state
   virtual void update() {}

   virtual void run();

   int getStatus() { OpenThreads::ScopedLock<OpenThreads::Mutex> ml(m_status_mutex); int i = m_status; return i; }


protected:

   void setStatus(int i) { OpenThreads::ScopedLock<OpenThreads::Mutex> ml(m_status_mutex); m_status=i; }

  /// Destructor
  ~ThreadedSensor();


private:


  
  const char *className() { return "ThreadedSensor"; }

  OpenThreads::Mutex m_io_mutex;
  osgSensor::Event m_ready_event;

  osg::ref_ptr<Sensor> m_sensor;

  struct StationData {
    osg::Vec3 position;
    osg::Quat orientation;
    bool stylus;
    int frame;
  };


  typedef std::vector< StationData > SensorData;
  SensorData m_shared_data;
  void updateSharedData( const SensorData& data );

  int m_current_frame;
  mutable OpenThreads::Mutex m_status_mutex;
  int m_status;
	bool m_shutting_down;

};

} // Namespace sensors
#endif

