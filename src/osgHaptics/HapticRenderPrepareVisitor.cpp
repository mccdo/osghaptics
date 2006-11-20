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
#include <osgHaptics/HapticRenderPrepareVisitor.h>

#include <osg/Object>
#include <osg/Geode>
#include <osg/Drawable>

using namespace osgHaptics;

HapticRenderPrepareVisitor::HapticRenderPrepareVisitor(HapticDevice *device, TraversalMode tm) :
 NodeVisitor(tm), m_device(device)
{
  if (!device)
    throw std::runtime_error("HapticRenderPrepareVisitor::HapticRenderPrepareVisitor(): Device pointer is NULL!");
}

void HapticRenderPrepareVisitor::apply(osg::Geode& node)
{
  // iterate over all drawables.

  for (unsigned int i=0; i < node.getNumDrawables(); i++)
  {
    osg::Drawable *drawable = node.getDrawable(i);
    
    osg::StateSet *ss = drawable->getOrCreateStateSet();
    // Check if there are already a shape attached to this drawable
    osg::StateAttribute *sa = ss->getAttribute(Shape::getSAType());
    if (!sa) {
      // attach a shape
      Shape *shape = new Shape(m_device.get());

      ss->setAttributeAndModes(shape, osg::StateAttribute::ON);
      if (!m_shape.valid())
      {
        m_shape = new ShapeComposite(m_device.get());
      }
      m_shape->addChild(shape);
    }
    else {

      osg::notify(osg::WARN) << std::endl << "Drawables are instanced, which means that problem will occur" << std::endl;
      osg::notify(osg::WARN) << "Each Drawable can have only one haptic Shape attached to it." << std::endl;
      osg::notify(osg::WARN) << "and we cant draw to the same haptic shape multiple times. This means that only " << std::endl;
      osg::notify(osg::WARN) << "the first occurence of the drawable will been drawn haptically. " << std::endl;
      osg::notify(osg::WARN) << "Consider regenerating the scene and remove instanced drawables making each one unique." << std::endl << std::endl;
    }

    // collect all the shapes into a ShapeComposite
  }
}
