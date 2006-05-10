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

#include <osg/io_utils>
#include <osg/Timer>
#include <osgSensor/ThreadedSensor.h>
#include <osg/Notify>

using namespace osgSensor;

ThreadedSensor::ThreadedSensor(Sensor *sensor) : m_sensor(sensor), m_shutting_down(false)
{
  m_ready_event.reset();

  setStatus(1); // Ok
  start();
  m_shared_data.resize(m_sensor->getNumberOfSensors());
}

bool ThreadedSensor::init( unsigned int num_sensors )
{
  if (num_sensors != m_sensor->getNumberOfSensors()) {
    throw std::runtime_error("ThreadSensor::init Number of sensors specified does not match the used number of sensors available");
  }
  return true;
}
   
   
int ThreadedSensor::read( std::ostream& ostr, unsigned long timeout )
{
  if (!getStatus())
    return 0;

  if (!m_ready_event.wait(timeout)) {
    osg::notify(osg::WARN) <<  "Timeout waiting for sensor data" << std::endl;
    return 0;
  }

  return 0;
}

int ThreadedSensor::read(unsigned int sensor_no, osg::Vec3& p, osg::Quat& q, unsigned long timeout )
{
  if (!getStatus())
    return 0;

  if (!m_ready_event.wait(timeout)) {
    osg::notify(osg::WARN) << "Timeout waiting for sensor data" << std::endl;
    return 0;
  }


  if (sensor_no > m_sensor->getNumberOfSensors()) {
    osg::notify(osg::WARN) << "Invalid sensor number specified: " << sensor_no << std::endl;
    return 0;
  }

  OpenThreads::ScopedLock<OpenThreads::Mutex> locker(m_io_mutex);
  p = m_shared_data[sensor_no-1].position;
  q = m_shared_data[sensor_no-1].orientation;
  return 1;
}


void ThreadedSensor::shutdown( void )
{
  m_shutting_down = true;
	m_ready_event.signal();
  stop();


  if (!wait(500)) {
    osg::notify(osg::WARN) << "ThreadedSensor::~ThreadedSensor: Unable to kill the running thread" << std::endl;
    terminate();
  }
  
}
ThreadedSensor::~ThreadedSensor()
{
  shutdown();
}

void ThreadedSensor::run()
{
  SensorData data;
  data.resize(m_sensor->getNumberOfSensors());
  int n=0;
  osg::Timer_t begin = osg::Timer::instance()->tick();

  while(1) {
    n++;
    for(unsigned int i=0; i < m_sensor->getNumberOfSensors(); i++) {
      if (m_shutting_down)
				return;

			if (!m_sensor->read(i+1, data[i].position, data[i].orientation))
      {
        osg::notify(osg::WARN)<< "ThreadedSensor::run(): Error reading from sensor" << std::endl;
        setStatus(0);
      }
      else 
        setStatus(1);
    }

    updateSharedData(data);

    // Should we continue?
    Continue();
    osg::Timer_t start = osg::Timer::instance()->tick();
    OpenThreads::Thread::microSleep(1*1000);
    osg::Timer_t s = osg::Timer::instance()->tick();
    double t = osg::Timer::instance()->delta_m(start,s);
    double d = osg::Timer::instance()->delta_s(begin, s);
  }
}

void ThreadedSensor::updateSharedData(const SensorData& data)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> locker(m_io_mutex);
  
  for(unsigned int i=0; i < m_sensor->getNumberOfSensors(); i++) {
    m_shared_data[i] = data[i];
  }
  
  //  We now have data available
  m_ready_event.signal();  
}
