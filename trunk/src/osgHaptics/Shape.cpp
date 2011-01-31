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

Shape::Shape(HapticDevice *device, const std::string& name) : m_name(name), m_enabled(1),
  m_device(device)
{

	//register this device for this shape
  addDevice(device);

}

Shape::Shape(HapticDevice *device, int enabled) : m_name("no name"), m_enabled(enabled),
  m_device(device)
{

  //register this device for this shape
  addDevice(device);
}


//--by SophiaSoo/CUHK: for two arms, NEW CONSTRUCTOR
Shape::Shape() : m_name("no name"), m_shape_id(0) , m_enabled(1) {}


Shape::~Shape()
{
  //--by SophiaSoo/CUHK: for two arms
  for (unsigned int i=0; i<m_devices.size(); i++) {
  	// Make sure the device is valid, could be that this is called late when
		// osg is cleaning up the scenegraph, after the device has ben shutdown
		// To avoid crash we ignore to remove the shape.
		// A better way is of course to shut down the device after all shapes is gone!
		if (!m_devices[i].valid() || !m_devices[i]->isInitialized()) 
			continue;

		m_devices[i]->unRegisterContactEventHandler(this);
		m_devices[i]->makeCurrent();
		hlDeleteShapes(m_shape_ids[i], 1);
  } //for

}

void Shape::preDraw() const
{
  //--by SophiaSoo/CUHK: for two arms
  int idx = getCurrentDeviceIndex();
  if (isValidIndex(idx)) {
 		if (m_enabled && m_devices[idx].valid() && m_devices[idx]->getEnableShapeRender()) {
			hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, m_shape_ids[idx]);
		}
  } //if

}

void Shape::postDraw() const 
{ 

  //--by SophiaSoo/CUHK: for two arms
  int idx = getCurrentDeviceIndex();
  if (isValidIndex(idx)) {
 		if (m_enabled && m_devices[idx].valid() && m_devices[idx]->getEnableShapeRender()) {
			hlEndShape(); 
		}
  } //if

}

//--by SophiaSoo/CUHK: for two arms, NEW FUNCTION
void Shape::addDevice(HapticDevice *device) 
{
  device->makeCurrent();

  // First check if 'device' is already present in the list of devices
	// if so, dont add it again, because that will only make the app crash
	// when hl encounters the same Shape used twice
	bool found = false;
	for(unsigned int i=0;i < m_devices.size(); i++) {
		if (m_devices[i]==device)
			found = true;
	}
	
	if (found)
		return;

	// Create a shape id for this device context
  m_shape_id = hlGenShapes(1);

  HLerror error = hlGetError();
  if (error.errorCode == HL_INVALID_OPERATION) throw std::runtime_error("Haptic::Shape: No shape id´s available");  

  //store child
	m_devices.push_back(device);
  m_shape_ids.push_back(m_shape_id);

}


//Return the index of the current device
int Shape::getCurrentDeviceIndex() const {
	HHD hHD = hdGetCurrentDevice();
	for (unsigned int i=0; i<m_devices.size(); i++) {
		if (m_devices[i].valid() && hHD == m_devices[i]->getHandle()) {
			return i ;
		}//if
	} //for
	return -1;
}
