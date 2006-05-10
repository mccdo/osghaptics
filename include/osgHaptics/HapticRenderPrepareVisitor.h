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

#ifndef __HapticRenderPrepareVisitor_h__
#define __HapticRenderPrepareVisitor_h__

#include <osg/NodeVisitor>

#include <osgHaptics/export.h>
#include <osgHaptics/RenderTriangleOperator.h>
#include <osgHaptics/Shape.h>
#include <osgHaptics/ShapeComposite.h>
#include <osg/observer_ptr>


  namespace osgHaptics {
  

  /// Class that attach a new haptic shape for each drawable found in the tree. 
  
  /*!
    This class traverses all drawables and attaches a new haptic shape for each drawable it find.
    All created shapes will be added to a ShapeComposite which will be the parent.
    For contact detection this ShapeComposite can be used.

    If the subgraph contains instances, that is, a drawable occurs on several positions in the tree, then
    only the first occurence of that drawable will actually be drawn haptically.
    The ShapeComposte can be accessed throught the getShape() method.
  */

  class OSGHAPTICS_EXPORT HapticRenderPrepareVisitor : public osg::NodeVisitor
  {
  public:
    HapticRenderPrepareVisitor(HapticDevice *device, osg::NodeVisitor::TraversalMode tm=osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);

    /// Destructor
    virtual ~HapticRenderPrepareVisitor() {}

    virtual void apply(osg::Geode& node);

    /*!
      Return the ShapeComposite that holds all the Shapes generated.
      IMPORTANT: For now, attach this to a ref_ptr somewhere, otherwise it will be dropped.
    */
    osgHaptics::Shape *getShape() { return m_shape.get(); }
    

  protected:
    osg::ref_ptr<ShapeComposite> m_shape;
    osg::observer_ptr<HapticDevice> m_device;

  };
} // namespace osgHaptics

#endif
