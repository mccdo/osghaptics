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

#ifndef __HapticRootNode_h__
#define __HapticRootNode_h__ 1


#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/NodeVisitor>
#include <osg/CopyOp>
#include <osgProducer/Viewer>

#include <osgHaptics/export.h>



  namespace osgHaptics {
  

  /// A node that calls HapticDevice::update during update traversal
  /**
  This can  be the parent of all haptical renderable nodes.
  All it does really is to set the already existing "HapticRenderBin" to be the current renderbin 
  for all its children.

  It also holds a reference to the HapticDevice which can be handy. So the device is not "lost". that is 
  dereferenced and deleted by ref_ptr´s 

  It is also possible to keep nodes that are supposed to be rendered into the "HapticRenderBin" in other places in
  the scenegraph too, just set the stateset->setRenderBinDetails(1, "HapticRenderBin"); for that node/group.

  The subgraph of this group will only be rendered once per frame to avoid problems in the Haptic Shape rendering.
  If a Haptic shape is rendered multiple times during one frame it will cause the application to crash.

  If viewer is specified in the constructor the scene will only be rendered in mono.
  */
  class OSGHAPTICS_EXPORT HapticRootNode : public osg::Group 
  {
  public:

    /*! Constructor

    \param viewer - If set, the subgraph will only be rendered in mono using a cullcallback
    */
    HapticRootNode( osgProducer::Viewer *viewer=0L );

    META_Node(osg ,Group);

    /*!
    Executed during traversal of the scenegraph.
    If the NodeVisitor is a Update visitor the internal state of the
    haptic device is updated.
    */
    virtual void traverse(osg::NodeVisitor &nv);

  protected:

    /// Destructor
    virtual ~HapticRootNode() {}


    /// Copy constructor
    HapticRootNode(const HapticRootNode &copy, const osg::CopyOp &copyop = osg::CopyOp::SHALLOW_COPY);

    /// Assignment operator
    HapticRootNode &operator=(const HapticRootNode &node); 

  private:
    int m_last_frame;
  };

} // namespace osgHaptics


#endif
