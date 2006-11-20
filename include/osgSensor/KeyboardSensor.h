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

#ifndef __KeyboardSensor_h__
#define __KeyboardSensor_h__

#include <osgSensor/OsgSensor.h>
#include <osgSensor/export.h>

namespace osgSensor {

  class  OSGSENSOR_EXPORT KeyboardSensor : public OsgSensor {
  public:

    /// Constructor. 
    KeyboardSensor();

    /*! 
      Take care of a keyboard event
      \param key - Pressed key
      \return true if key is handled otherwise false
    */
    virtual bool handle(int key);

    virtual ~KeyboardSensor(){};

  protected:
    

    /// Returns the name of this class
    virtual const char *className() { return "KeyboardSensor"; }

  private:

    /// Reads data from the sensor and updates all internal data
    virtual void sample();
  };
} // namespace sensors
#endif
