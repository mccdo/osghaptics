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

#ifndef __SensorLink_h__
#define __SensorLink_h__

#include <osgSensor/OsgSensor.h>
#include <osg/Referenced>

namespace osgSensor {

/// Class to build up a hierarchy of transformations for movement of objects/camera
/*!

*/
  class  OSGSENSOR_EXPORT SensorLink : public osg::Referenced  {
public: 
  /*! Constructor
  \param OsgSensor *sensor - pointer to a sensor that contains the active matrix
  */
  SensorLink(OsgSensor *sensor);

  /*! Constructor
  \param OsgSensor *sensor - pointer to a sensor that contains the active matrix
  */
  SensorLink();

  virtual ~SensorLink();


  /*! 
  Set the sensor of this link
  \param OsgSensor *sensor - pointer to a sensor that contains the active matrix
  */
  void sensor(OsgSensor *sensor) { m_sensor = sensor; }

  /*! 
  Get the sensor of this link
  \return OsgSensor *sensor - pointer to the sensor of this link
  */
  OsgSensor *sensor() { return m_sensor.get(); }

  /*!
  \param EnableFlag flag - Specifies what this manipulator will propagate to its dependencies
  Ex: if flag == EnablePosition then only position will be propagated from this manipulator
  */
  void setPropagate(OsgSensor::EnableFlag flag);

  /*!
  Calculates the active matrix: the matrix from the sensor concatenated with dependents matrices if any.
  \return the active matrix
  */
  osg::Matrix getMatrix();

  /// Add a dependent sensorlink
  void addDependent(SensorLink *);

  /// Remove a given SensorLink
  bool removeDependent(SensorLink *);

  /*!
    Update the internal state of the SensorLink, i.e. it really just
    calls the getMatrix method
  */
  void update() { getMatrix(); }

  /*! 
  Take care of a keyboard event
  \param key - Pressed key
  \return true if key is handled otherwise false
  */
  virtual bool handle(int key);


  /*!
  Enable/disables the sensor link
  If disabled, this SensorLink will not be updated with the current value of the sensor
  */
  void setEnable(bool flag) { m_enable = flag; }

  /*!
    Return true the SensorLink is enabled
  */
  bool getEnable() const { return m_enable; }

protected:


private:


  osg::ref_ptr<OsgSensor> m_sensor;
  osg::Matrix m_matrix;
  typedef std::vector< osg::ref_ptr<SensorLink> > DependentVector;
  typedef DependentVector::iterator DependentIterator;

  DependentVector m_dependents;
  bool m_enable;
};

} // namespace sensors
#endif

