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

#ifndef __osgHaptics_UpdateDeviceCallback_h__
#define __osgHaptics_UpdateDeviceCallback_h__

#include <osg/NodeCallback>
#include <osgHaptics/HapticDevice.h>
#include <osg/observer_ptr>


namespace osgHaptics {
  

class UpdateDeviceCallback : public osg::NodeCallback {

public:

  UpdateDeviceCallback(HapticDevice *device ) : m_device(device), m_last_traversal_number(-1) {}

  UpdateDeviceCallback(const UpdateDeviceCallback& nc,const osg::CopyOp&):
  m_device(nc.m_device) {}


  //META_Object(osgHaptics,UpdateDeviceCallback);


  /** Callback method called by the NodeVisitor when visiting a node.*/
  virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
  { 
    if (nv->getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR) {

      // Make sure we only execute this once during this frame.
      // could be two or more for stereo/multipipe...
      if ( nv->getTraversalNumber() != m_last_traversal_number && nv->getFrameStamp())
      {
        m_last_traversal_number = nv->getTraversalNumber();
        if (m_device.valid())
          m_device->update();
      }
    } 

    
    traverse(node,nv);
  }

protected:

  osg::observer_ptr<HapticDevice> m_device;
  int m_last_traversal_number;
  virtual ~UpdateDeviceCallback() {}
};
} // namespace osgHaptics



#endif
