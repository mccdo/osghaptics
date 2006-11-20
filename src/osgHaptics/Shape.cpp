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


#include <osgHaptics/HapticDevice.h>
#include <osgHaptics/Shape.h>
#include <stdexcept>
#include <osg/Notify>
#include <osg/Matrix>
#include <osgHaptics/types.h>

#include <iostream>


using namespace osgHaptics;


//Shape::Shape(int generate_id) :   m_valid(false)
//{
//  m_shape_id = 0;
//}


Shape::Shape(HapticDevice *device, const std::string& name) : m_name(name), m_enabled(1),
  m_device(device)
{
  // Should be done per context!!
  m_shape_id = hlGenShapes(1);

  HLerror error = hlGetError();
  if (error.errorCode == HL_INVALID_OPERATION)
    throw std::runtime_error("Haptic::Shape: No shape id´s available");  
}

Shape::Shape(HapticDevice *device, int enabled) : m_name("no name"), m_enabled(enabled),
  m_device(device)
{
  // Should be done per context!!
  m_shape_id = hlGenShapes(1);

  HLerror error = hlGetError();
  if (error.errorCode == HL_INVALID_OPERATION)
    throw std::runtime_error("Haptic::Shape: No shape id´s available");  
}


Shape::~Shape()
{

  if (m_device.valid())
    m_device->unRegisterContactEventHandler(this);

  //if (m_valid)
    hlDeleteShapes(m_shape_id, 1);
}

void Shape::preDraw() const
{
  if (m_enabled && m_device->getEnableShapeRender()) {
    hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, m_shape_id);
  }
}

void Shape::postDraw() const 
{ 
  if (m_enabled && m_device.valid() && m_device->getEnableShapeRender()) {
    hlEndShape(); 
  }
}
