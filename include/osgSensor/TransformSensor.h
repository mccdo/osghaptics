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

#ifndef __TransformSensor_h__
#define __TransformSensor_h__

#include <osg/MatrixTransform>
#include <osgSensor/OsgSensor.h>
#include <osgSensor/export.h>

namespace osgSensor {


  class OSGSENSOR_EXPORT TransformSensor : public OsgSensor {
  public:

    /*
      Constructor.
      \param transform - Which MatrixTransform node to read data from
    */
    TransformSensor(osg::MatrixTransform *transform);

    /*! 
      Take care of a keyboard event
      \param key - Pressed key
      \return true if key is handled otherwise false
    */
    virtual bool handle(int key);

    virtual ~TransformSensor(){};

    /// Return the transformationnode that are feeding transformation to this sensor
    osg::MatrixTransform *getTransform() { return m_transform.get(); }

    /// Return the transformationnode that are feeding transformation to this sensor
    const osg::MatrixTransform *getTransform() const { return m_transform.get(); }

  protected:

  private:

    osg::ref_ptr<osg::MatrixTransform> m_transform;

    /// Reads data from the sensor and updates all internal data
    virtual void sample();
  };
} // namespace

#endif
