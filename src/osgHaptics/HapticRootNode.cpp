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
#include <osgHaptics/HapticRootNode.h>
#include <osgHaptics/HapticRenderBin.h>
#include <osgHaptics/MonoCullCallback.h>

using namespace osgHaptics;

HapticRootNode::HapticRootNode(osgUtil::SceneView *sceneView) : Group(), m_last_frame(-1)

{


  // All drawables in this subgraph will be rendered using the HapticRenderbin
  this->getOrCreateStateSet()->setRenderBinDetails(1, "HapticRenderBin");//, osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);


  // Dont cull nor optimize this group away.
  setCullingActive(false);
  setDataVariance(osg::Object::DYNAMIC);

  if (sceneView) {
    setCullCallback(new MonoCullCallback(sceneView));

//    osgProducer::OsgSceneHandler* sceneHandler = viewer->getSceneHandlerList().front().get();
//    osgUtil::SceneView *sceneView = sceneHandler->getSceneView();

  }

}

HapticRootNode & HapticRootNode::operator=(const HapticRootNode &node)
{ 
  if (this == &node) return *this; 

  return *this;
}


HapticRootNode::HapticRootNode(const HapticRootNode &copy, const osg::CopyOp &copyop)
  : osg::Group(copy, copyop)
{
  *this = copy;
}


// Only traverse the culling once. (To avoid problems with the haptic rendering later on)
void HapticRootNode::traverse(osg::NodeVisitor &nv)
{
  const osg::FrameStamp *fs = nv.getFrameStamp();
  int frame=0;
  if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR && fs && (frame = nv.getFrameStamp()->getFrameNumber()) == m_last_frame) {
    //--by SophiaSoo/CUHK: for two arms
    //return;
  }

  m_last_frame = frame;
  Group::traverse(nv);
}
