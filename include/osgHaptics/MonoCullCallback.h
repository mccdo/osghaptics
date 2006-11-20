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

#ifndef __osghaptics_MONOCULLCALLBACK_h__
#define __osghaptics_MONOCULLCALLBACK_h__

#include <osg/NodeCallback>
#include <osgUtil/SceneView>
#include <osgUtil/CullVisitor>
#include <osg/Matrix>


  namespace osgHaptics {
  

/// Class to make sure subgraph is rendered only in mono
/*!
  This callback will setup the projection and viewmatrix so that the scene
  is rendered in mono.
  Observe that any osg::Projection or anything else that changes the projection
  matrix above this node will be ignored.
*/ 

class MonoCullCallback : public osg::NodeCallback {
public:
  MonoCullCallback(osgUtil::SceneView *sv) : m_sceneview(sv) {}
  virtual void operator()(osg::Node *node, osg::NodeVisitor* nv){
    bool pushed = false;
    osgUtil::CullVisitor *cv=0L;
    if (nv->getVisitorType() == osg::NodeVisitor::CULL_VISITOR) {
      cv = dynamic_cast<osgUtil::CullVisitor *>(nv);
      if (cv) {
        osg::Matrix pm = m_sceneview->getProjectionMatrix();
        osg::Matrix mv = m_sceneview->getViewMatrix();
        cv->pushProjectionMatrix(new osg::RefMatrix(pm));
        cv->pushModelViewMatrix(new osg::RefMatrix(mv));
        pushed = true;
      }
    }

    traverse(node,nv);

    if (pushed) {
      cv->popProjectionMatrix();
      cv->popModelViewMatrix();

    }
  }
protected:
  osg::ref_ptr<osgUtil::SceneView> m_sceneview;
};
} // namespace osghaptics


#endif
