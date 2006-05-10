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

#ifndef __HapticSpringNode_h__
#define __HapticSpringNode_h__ 1


#include <osg/ref_ptr>
#include <osg/Transform>
#include <osg/NodeVisitor>
#include <osg/CopyOp>

#include <osgHaptics/export.h>
#include <osgHaptics/SpringForceOperator.h>
#include <osgHaptics/HapticDevice.h>



  namespace osgHaptics {

    /// 
    /**
    */
    class OSGHAPTICS_EXPORT HapticSpringNode : public osg::Transform 
    {
    public:

      /// Default constructor
      HapticSpringNode(HapticDevice *device);
      HapticSpringNode() {}


      META_Node(osgHaptics ,HapticSpringNode);

      osgHaptics::SpringForceOperator *getForceOperator() { return m_force_operator.get(); }

      void setEnable(bool f) { m_force_operator->setEnable(f); }
      bool getEnable() const { return m_force_operator->getEnable(); }

      /*!
      Executed during traversal of the scenegraph.
      If the NodeVisitor is a Update visitor the internal state of the
      haptic device is updated.
      */
      virtual void traverse(osg::NodeVisitor &nv);

    protected:

      /// Destructor
      virtual ~HapticSpringNode();


      /// Copy constructor
      HapticSpringNode(const HapticSpringNode &copy, const osg::CopyOp &copyop = osg::CopyOp::SHALLOW_COPY);

      /// Assignment operator
      HapticSpringNode &operator=(const HapticSpringNode &node); 

    private:
      osg::ref_ptr<SpringForceOperator> m_force_operator; 
      osg::ref_ptr<HapticDevice> m_device; 
    };

  } // namespace osgHaptics


#endif
