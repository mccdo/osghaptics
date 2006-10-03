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

#include <iostream>
#include <osgHaptics/HapticRenderBin.h>

using namespace osgHaptics;

osgUtil::RegisterRenderBinProxy s_registerRenderBinProxy("HapticRenderBin",new HapticRenderBin(osgUtil::RenderBin::getDefaultRenderBinSortMode()));

HapticRenderBin::HapticRenderBin(SortMode mode) : RenderBin(mode), m_last_frame(-1)
{

}


HapticRenderBin::HapticRenderBin(const HapticRenderBin& rhs,const osg::CopyOp& copyop):
RenderBin(rhs,copyop), m_haptic_renderleaf(rhs.m_haptic_renderleaf), m_last_frame(rhs.m_last_frame)

{
}

HapticRenderBin::HapticRenderBin() : RenderBin(), m_last_frame(-1)
{
}

const osgHaptics::Shape *HapticRenderBin::getShape(osg::State& state) const
{
  const osg::StateAttribute *sa = state.getLastAppliedAttribute(osgHaptics::Shape::getSAType());

  if (sa) {
    const osgHaptics::Shape *shape = static_cast<const osgHaptics::Shape*> (sa);
    return shape;
  }

  return 0L;
}

bool HapticRenderBin::hasBeenDrawn(osg::State& state)
{

  // Is this a haptic drawable? That is, does it has a Shape attached as StateAttribute?
  const osgHaptics::Shape *shape = getShape(state);
  if (shape) {

    // Its a haptic shape, lets see if it has been rendered before.
    if (m_rendered_shapes.find(shape) != m_rendered_shapes.end()) {
      return true; // It has been rendered before, dont do it again.
    }
    else {
      m_rendered_shapes[shape] = shape; // NOw it has been rendered, so add it.
      return false;
    }      
  } 

  // No haptic shape attached, just render it.   
  return false; 
}

void HapticRenderBin::drawImplementation(osg::RenderInfo& renderInfo,osgUtil::RenderLeaf*& previous)
{

  // Only render once per frame. Important to keep track of during stereo rendering
  if (m_last_frame == renderInfo.getState()->getFrameStamp()->getFrameNumber())
    return;

  m_last_frame = renderInfo.getState()->getFrameStamp()->getFrameNumber();


  // Clear the list of already drawn drawables 
  m_rendered_shapes.clear();

  // For each drawn drawable that has a haptic Shape StateAttribute attached to it,
  // store a weak reference and before rendering successive drawables, check if it has already been drawn...

  // draw first set of draw bins.
  osgUtil::RenderBin::RenderBinList::iterator rbitr;
  for(rbitr = _bins.begin();
    rbitr!=_bins.end() && rbitr->first<0;
    ++rbitr)
  {
    rbitr->second->draw(renderInfo ,previous);
  }


  // draw fine grained ordering.
  for(osgUtil::RenderBin::RenderLeafList::iterator rlitr= _renderLeafList.begin();
    rlitr!= _renderLeafList.end();
    ++rlitr)
  {
    osgUtil::RenderLeaf* rl = *rlitr;
    renderHapticLeaf(rl, renderInfo, previous);
    //rl->render(state,previous);
    previous = rl;
  }


  // draw coarse grained ordering.
  for(osgUtil::RenderBin::StateGraphList::iterator oitr=_stateGraphList.begin();
    oitr!=_stateGraphList.end();
    ++oitr)
  {

    for(osgUtil::StateGraph::LeafList::iterator dw_itr = (*oitr)->_leaves.begin();
      dw_itr != (*oitr)->_leaves.end();
      ++dw_itr)
    {
      osgUtil::RenderLeaf* rl = dw_itr->get();
      renderHapticLeaf(rl,  renderInfo, previous);
//      rl->render(state,previous);
      previous = rl;

    }
  }


  // draw post bins.
  for(;
    rbitr!=_bins.end();
    ++rbitr)
  {
    rbitr->second->draw( renderInfo,previous);
  }
}

HapticRenderBin::~HapticRenderBin()
{
}


void HapticRenderBin::renderHapticLeaf(osgUtil::RenderLeaf* original, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf *previous) 
{
  if (!m_haptic_renderleaf.valid())
    m_haptic_renderleaf = new HapticRenderLeaf(original, this);
  else
    m_haptic_renderleaf->set(original);
  m_haptic_renderleaf->render(renderInfo, previous);
}
