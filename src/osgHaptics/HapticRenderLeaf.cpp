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


#include <osgHaptics/HapticRenderLeaf.h>
#include <osgHaptics/RenderTriangleOperator.h>
#include <osgHaptics/HapticRenderBin.h>
#include <osgHaptics/Shape.h>

#include <osgUtil/StateGraph>
#include <osg/Geometry>
#include <osg/TriangleFunctor>

using namespace osgHaptics;




void HapticRenderLeaf::render(osg::RenderInfo& renderInfo,osgUtil::RenderLeaf* previous)
{
  // don't draw this leaf if the abort rendering flag has been set.
  if (renderInfo.getState()->getAbortRendering())
  {
    //cout << "early abort"<<endl;
    return;
  }

  if (previous)
  {

    // apply matrices if required.
    renderInfo.getState()->applyProjectionMatrix(_projection.get());
    renderInfo.getState()->applyModelViewMatrix(_modelview.get());

    // apply state if required.
    osgUtil::StateGraph* prev_rg = previous->_parent;
    osgUtil::StateGraph* prev_rg_parent = prev_rg->_parent;
    osgUtil::StateGraph* rg = _parent;
    if (prev_rg_parent!=rg->_parent)
    {
      osgUtil::StateGraph::moveStateGraph(*renderInfo.getState(),prev_rg_parent,rg->_parent);

      // send state changes and matrix changes to OpenGL.
      renderInfo.getState()->apply(rg->_stateset);

    }
    else if (rg!=prev_rg)
    {

      // send state changes and matrix changes to OpenGL.
      renderInfo.getState()->apply(rg->_stateset);

    }

    
    const osgHaptics::Shape *shape = m_renderbin->getShape(renderInfo);      
    bool render_shape=false;

    render_shape = !m_renderbin->hasBeenDrawn(renderInfo);
  
    // If we have a shape,
    // and the device is reporting, Dont render haptic shape,
    // then bail out and skip the rendering of this HapticRenderLeaf
    if (shape && !shape->getHapticDevice()->getEnableShapeRender())
      return;

    if (shape && render_shape) {
      //shape = static_cast<const osgHaptics::Shape*> (sa);
      shape->preDraw();      
    }

    osg::Geometry* geom = dynamic_cast<osg::Geometry *>(_drawable);
    if (geom) {
      RenderTriangleOperator op;
      geom->accept(op);
    }
    else
      // draw the drawable
      _drawable->draw(renderInfo);
    
    if (shape && render_shape) 
      shape->postDraw();
  }
  else
  {
    std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    // apply matrices if required.
    renderInfo.getState()->applyProjectionMatrix(_projection.get());
    renderInfo.getState()->applyModelViewMatrix(_modelview.get());

    // apply state if required.
    osgUtil::StateGraph::moveStateGraph(*renderInfo.getState(),NULL,_parent->_parent);

    renderInfo.getState()->apply(_parent->_stateset);

    // draw the drawable
    _drawable->draw(renderInfo);
  }
}
