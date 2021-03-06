/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Sensor Library
* Copyright (C) 2006 VRlab, Ume� University
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

ThreadedSensor::ThreadedSensor(Sensor *sensor, const std::string& name ) : Sensor(name), m_sensor(sensor)
{
  m_ready_read_event.reset();

  setStatus(1); // Ok
  m_shared_data.resize(m_sensor->getNumberOfSensors());
  start();
}

  
   
int ThreadedSensor::read( std::ostream& ostr, unsigned long timeout )
{

	if (!getStatus())
    return 0;

  if (!m_ready_read_event.block(timeout)) {
    osg::notify(osg::WARN) <<  "Timeout waiting for sensor data" << std::endl;
    return 0;
  }

  return 0;
}

int ThreadedSensor::read(unsigned int sensor_no, osg::Vec3& p, osg::Quat& q, unsigned long timeout )
{

  if (!getStatus())
    return 0;


  if (!m_ready_read_event.block(timeout)) {
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


void ThreadedSensor::shutdown( float t )
{

  m_ready_read_event.release();
  stop();

  if (isRunning() && !wait(2000)) {
    //osg::notify(osg::WARN) << "ThreadedSensor::shutdown: Unable to kill the running thread" << std::endl;
		cancel();
		//    terminate();
  }

  if (m_sensor.valid())
	  m_sensor->shutdown(t);

}


unsigned int 
ThreadedSensor::getNumberOfButtons()
{
	if (m_sensor.valid())
		return m_sensor->getNumberOfButtons();
	else
		return 0;
}


unsigned int 
ThreadedSensor::getNumberOfValuators()
{
	if (m_sensor.valid())
		return m_sensor->getNumberOfValuators();
	else
		return 0;
}

ThreadedSensor::~ThreadedSensor()
{
  //shutdown(0.0f);
}


void ThreadedSensor::cancelCleanup()
{


}

void ThreadedSensor::run()
{
  SensorData data;
  data.resize(m_sensor->getNumberOfSensors());
  int n=0;
  // Unused
  //osg::Timer_t begin = osg::Timer::instance()->tick();
  //double t = 0.0;
	osg::Timer_t start; 
    // unused
    //osg::Timer_t stop=osg::Timer::instance()->tick();
  while(1) {
		start = osg::Timer::instance()->tick();
    n++;

		if (shouldStop()) {
			exit();
		}


		m_sensor->update(0.0f);

		for(unsigned int i=0; i < m_sensor->getNumberOfSensors(); i++) {

			if (shouldStop()) {
				exit();
			}

			if (!m_sensor->read(i+1, data[i].position, data[i].orientation))
			{
				osg::notify(osg::WARN)<< "ThreadedSensor::run(): Error reading from sensor" << std::endl;
				setStatus(0);
			}
			else 
				setStatus(1);
		}

		updateSharedData(data);

		if (shouldStop()) {
			std::cerr  << __FILE__ <<  " We should stop " << __LINE__ << std::endl;
			exit();
		}


		//osg::Timer_t start = osg::Timer::instance()->tick();
		OpenThreads::Thread::microSleep(1*1000);
		/*osg::Timer_t s = osg::Timer::instance()->tick();
		double t = osg::Timer::instance()->delta_m(start,s);
		double d = osg::Timer::instance()->delta_s(begin, s);
		*/
		n++;
	}
}

void ThreadedSensor::update(float t)
{
/*	if (m_sensor.valid())
		m_sensor->update(t);
		*/

}

void ThreadedSensor::updateSharedData(const SensorData& data)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> locker(m_io_mutex);
  
  for(unsigned int i=0; i < m_sensor->getNumberOfSensors(); i++) {
    m_shared_data[i] = data[i];
  }
  
  //  We now have data available
  m_ready_read_event.release();  
}
