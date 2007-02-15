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
#include <osgProducer/Viewer>

#include <Producer/RenderSurface>



  namespace osgHaptics {


/// Callback to be attached to camera rendering haptics view. Will start haptic rendering frame
class HapticDevicePreRenderCallback: public Producer::Camera::Callback
{
public:

  HapticDevicePreRenderCallback(osgHaptics::HapticDevice *device): 
      m_device(device) {}

      virtual void operator()( const Producer::Camera & camera)
      {
        const Producer::RenderSurface *rs = camera.getRenderSurface();
        m_device->updateWorkspace(rs->getWindowWidth(), rs->getWindowHeight());

	    //--by SophiaSoo/CUHK: for two arms
        m_device->makeCurrentDevice();

        m_device->beginFrame();
      }

protected:

  osg::ref_ptr<osgHaptics::HapticDevice> m_device;    
};


/// Callback to be attached to camera rendering haptics view. Will stop haptic rendering frame.
class HapticDevicePostRenderCallback: public Producer::Camera::Callback
{
public:

  HapticDevicePostRenderCallback(osgHaptics::HapticDevice *device): 
      m_device(device) {}

      virtual void operator()( const Producer::Camera & camera)
      {
        m_device->endFrame();
      }

protected:

  osg::ref_ptr<osgHaptics::HapticDevice> m_device;

};

/// Utility function to attach pre and post draw operations o the default camera
void prepareHapticCamera(Producer::Camera *camera, HapticDevice *device, osg::Node *scene=0L) {
  camera->addPreDrawCallback(new osgHaptics::HapticDevicePreRenderCallback(device));
  camera->addPostDrawCallback(new osgHaptics::HapticDevicePostRenderCallback(device));

  // Get bounding box of the scene

  if (scene) {

    osg::BoundingSphere bs = scene->getBound();

    float radius = bs.radius();
    //std::cerr << "Radius: " << radius << std::endl;

    double left, right, bottom, top, nearclip, farclip;
    camera->getLens()->getParams(left, right, bottom, top, nearclip, farclip);
    //std::cerr << "left: " << left << " right: " << right << " bottom: " << bottom << " top: " << top << " nearclip: " << nearclip << " farclip: " << farclip << std::endl;
    camera->getLens()->setAutoAspect(false); 
    camera->getLens()->setFrustum(left, right, bottom, top, nearclip, radius*1.3); 
    camera->getLens()->getParams(left, right, bottom, top, nearclip, farclip);
    //std::cerr << "left: " << left << " right: " << right << " bottom: " << bottom << " top: " << top << " nearclip: " << nearclip << " farclip: " << farclip << std::endl;
  }
}

} // namespace osgSensors

#endif
