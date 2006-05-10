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

#ifndef __HapticShapeGroup__
#define __HapticShapeGroup__

#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Group>

#include <osgHaptics/export.h>
#include <osgHaptics/Material.h>
#include <osgHaptics/Shape.h>
#include <osgHaptics/types.h>





  namespace osgHaptics {
      

///
    class OSGHAPTICS_API HapticShapeGroup : public osg::Group
{
public:

  class HapticPreRenderDrawable : public osg::Drawable {
  public:

    // Set general settings for this class with macro
    META_Object(osgHaptics, HapticPreRenderDrawable);

    HapticPreRenderDrawable(osgHaptics::Shape *shape) : m_shape(shape)
    {

      setSupportsDisplayList(false);
    }
    virtual void drawImplementation(osg::State& state) const
    {
      m_shape->preDraw();
    }

    /** Compute bounding box (axis aligned) */
    virtual osg::BoundingBox computeBound() const
    {
      _boundingBox.set(osg::Vec3(-1,-1,-1), osg::Vec3(1,1,1));
      return _boundingBox;
    }      

  protected:

    HapticPreRenderDrawable(const HapticPreRenderDrawable& node, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) {}
    HapticPreRenderDrawable() {}

    /// Destructor
    virtual ~HapticPreRenderDrawable(){}

  private:
    osg::ref_ptr<osgHaptics::Shape> m_shape;
  };


  ///
  class HapticPostRenderDrawable : public osg::Drawable {
  public:

    // Set general settings for this class with macro
    META_Object(osgHaptics, HapticPostRenderDrawable);


    HapticPostRenderDrawable(osgHaptics::Shape *shape) : m_shape(shape)
    {
      setSupportsDisplayList(false);
    }
    virtual void drawImplementation(osg::State& state) const
    {
      m_shape->postDraw();
    }

    /** Compute bounding box (axis aligned) */
    virtual osg::BoundingBox computeBound() const
    {

      _boundingBox.set(osg::Vec3(-1,-1,-1), osg::Vec3(1,1,1));
      return _boundingBox;
    }      
  protected:
    HapticPostRenderDrawable(const HapticPostRenderDrawable& node, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) {}
    HapticPostRenderDrawable() {}
    
    /// Destructor
    virtual ~HapticPostRenderDrawable(){}

  private:
    osg::ref_ptr<osgHaptics::Shape> m_shape;
  };

  ///
  class HapticRenderGeode : public osg::Geode {
  public:
    HapticRenderGeode()
    {
      // Dont cull this haptic node
      setCullingActive(false); 
    }



  protected:
    
    /// Destructor
    virtual ~HapticRenderGeode() {}

  };


public: 

  // Set general settings for this class with macro
  META_Object(osgHaptics, HapticShapeGroup);

  HapticShapeGroup(const HapticShapeGroup& node, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) {}

  /// Default constructor, create a new default shape
  HapticShapeGroup() {
    m_shape = new osgHaptics::Shape();
    init();
  }

  /// Constructor
  HapticShapeGroup(osgHaptics::Shape *shape) : m_shape(shape) 
  {
    init();
  }

    // Always insert children between the pre and post drawables
    virtual bool addChild(osg::Node *node) {
      Group::insertChild(this->getNumChildren()-1, node);
      return true;
    }

    void setHapticShape(osgHaptics::Shape *shape) { m_shape = shape; }
    osgHaptics::Shape *getHapticShape() { return m_shape.get(); }

  private:
    void init();

    /// Destructor
    virtual ~HapticShapeGroup() {}

    osg::ref_ptr<osgHaptics::Shape> m_shape;
  };


  class HapticDrawable : public osg::Drawable {
  public:

    // Set general settings for this class with macro
    META_Object(osgHaptics, HapticDrawable);

    HapticDrawable( osg::Drawable *drawable, Shape *shape) : m_drawable(drawable),
      m_shape(shape) {
      setUseDisplayList(false);
    }

    virtual void drawImplementation(osg::State& state) const
    {
      m_shape->preDraw();
      m_drawable->draw(state);
      m_shape->postDraw();
    }

    /** Compute bounding box (axis aligned) */
    virtual osg::BoundingBox computeBound() const
    {
      this->_boundingBox.init();

      _boundingBox.expandBy(osg::Vec3(-1,-1,-1));
      _boundingBox.expandBy(osg::Vec3(1,1,1));
      return _boundingBox;
    }      

  protected:

    HapticDrawable(){}
    HapticDrawable(const HapticDrawable& node, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) {
    }

    /// Destructor
    virtual ~HapticDrawable(){}

    osg::ref_ptr<Shape> m_shape;
    osg::ref_ptr<osg::Drawable> m_drawable;
  };



} // namespace osgHaptics

#endif
