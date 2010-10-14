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

#ifndef __HAPTIC_RENDERLEAF_H__
#define __HAPTIC_RENDERLEAF_H__

#include <osgHaptics/export.h>
#include <osgUtil/RenderLeaf>
#include <osg/Matrix>
#include <osg/observer_ptr>




  namespace osgHaptics {
  
  class HapticRenderBin;

  /** Container class for all data required for rendering of drawables.
  */
    class OSGHAPTICS_EXPORT HapticRenderLeaf : public osgUtil::RenderLeaf
  {
  public:

#ifdef OSGUTIL_RENDERBACKEND_USE_REF_PTR
    inline HapticRenderLeaf(osgUtil::RenderLeaf *leaf, HapticRenderBin *renderbin) : 
    RenderLeaf(leaf->_drawable.get(), leaf->_projection.get(), leaf->_modelview.get(), leaf->_depth),
      m_renderbin(renderbin)
    {
      _parent=leaf->_parent;
    }

    inline void set(osgUtil::RenderLeaf *leaf) {
      RenderLeaf::set(leaf->_drawable.get(), leaf->_projection.get(), leaf->_modelview.get(), leaf->_depth);
      _parent=leaf->_parent;
    }

#else
    inline HapticRenderLeaf(osgUtil::RenderLeaf *leaf, HapticRenderBin *renderbin) : 
      RenderLeaf(leaf->_drawable, leaf->_projection.get(), leaf->_modelview.get(), leaf->_depth),
        m_renderbin(renderbin)
    {
      _parent=leaf->_parent;
    }
    
    inline void set(osgUtil::RenderLeaf *leaf) {
      RenderLeaf::set(leaf->_drawable, leaf->_projection.get(), leaf->_modelview.get(), leaf->_depth);
      _parent=leaf->_parent;
    }
#endif
    virtual void render(osg::RenderInfo& renderInfo, osgUtil::RenderLeaf* previous);

  protected:

    /// Destructor
    virtual ~HapticRenderLeaf() {}

  private:
    osg::observer_ptr<HapticRenderBin> m_renderbin;

  };

} // namespace osgHaptics

#endif
