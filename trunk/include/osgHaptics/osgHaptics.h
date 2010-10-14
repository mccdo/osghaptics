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

#ifndef __OSG_HAPTICS_H__
#define __OSG_HAPTICS_H__

#include <osgHaptics/HapticDevice.h>
#include <osgViewer/Viewer>
#include <osgHaptics/export.h>



namespace osgHaptics {


	/// Callback to be attached to camera rendering haptics view. Will start haptic rendering frame
	class OSGHAPTICS_EXPORT HapticDevicePreRenderCallback: public osg::Camera::DrawCallback
	{
	public:

		HapticDevicePreRenderCallback(osgHaptics::HapticDevice *device): 
				m_device(device) {}

				virtual void operator()( const osg::Camera & camera) const;

	protected:

		osg::ref_ptr<osgHaptics::HapticDevice> m_device;    
	};


	/// Callback to be attached to camera rendering haptics view. Will stop haptic rendering frame.
	class HapticDevicePostRenderCallback: public osg::Camera::DrawCallback
	{
	public:

		HapticDevicePostRenderCallback(osgHaptics::HapticDevice *device): 
				m_device(device) {}

				virtual void operator()( const osg::Camera & camera) const
				{
					m_device->endFrame();
				}

	protected:

		osg::ref_ptr<osgHaptics::HapticDevice> m_device;

	};

	/// Utility function to attach pre and post draw operations o the default camera
	void OSGHAPTICS_EXPORT prepareHapticCamera(osg::Camera *camera, HapticDevice *device, osg::Node *scene=0L);
} // namespace osgHaptics

#endif
