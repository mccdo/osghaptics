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


#ifndef __osgHaptics_ShapeComposite_h__
#define __osgHaptics_ShapeComposite_h__


#include <osgHaptics/Shape.h>
#include <map>




  namespace osgHaptics {

    /// Class that store a  Haptic shape id as a StateAttribute.

    /*!
    This class stores a shape id as a osg StateAttribute.
    When a drawable is rendered in the HapticRenderBin, the shape id of the current
    applied Shape will be used.
    */
    class OSGHAPTICS_EXPORT ShapeComposite : public Shape
    {
    public :

      typedef std::map<const Shape *, osg::ref_ptr<Shape> > ShapeMap;
      typedef std::map<HLuint, Shape *> ShapeIDMap;

      ShapeIDMap::iterator end() { return m_children_id.end(); }
      ShapeIDMap::iterator begin() { return m_children_id.begin(); }

      /// Default constructor
      ShapeComposite(HapticDevice *device, int enabled=1) : Shape(device) 
      { 
        setEnable(enabled ? true : false); 
      }

      ShapeComposite(HapticDevice *device, const std::string& name) : Shape(device, name) 
      {
        setEnable(false);
      }

      bool operator ==(const Shape& shape) const {
        return contains(&shape);
      }

      bool operator ==(const Shape *shape) const {
        return contains(shape);
      }

      virtual bool operator ==(HLuint shape_id)  {
        return contains(shape_id);
      }



      /// Enable the haptic rendering of this shape
      virtual void setEnable(bool flag);

      /** Copy constructor using CopyOp to manage deep vs shallow copy. */
      ShapeComposite(const ShapeComposite& trans,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
      Shape(trans,copyop)
      {}

        virtual osg::Object* cloneType() const { return new ShapeComposite(m_device.get(), 0); } 
        virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new ShapeComposite (*this,copyop); } 
        virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const ShapeComposite *>(obj)!=NULL; } 
        virtual const char* libraryName() const { return "osgHaptics"; } 
        virtual const char* className() const { return "ShapeComposite"; } 
        virtual Type getType() const { return osg::StateAttribute::Type(OSGHAPTICS_SHAPE); }

        /** Return -1 if *this < *rhs, 0 if *this==*rhs, 1 if *this>*rhs. */
        virtual int compare(const osg::StateAttribute& sa) const
        {
          // Check for equal types, then create the rhs variable
          // used by the COMPARE_StateAttribute_Paramter macros below.
          COMPARE_StateAttribute_Types(ShapeComposite,sa)

            // Compare each parameter in turn against the rhs.
            COMPARE_StateAttribute_Parameter(getNumChildren())

            return 0; // Passed all the above comparison macros, so must be equal.
        }

        /*!
        Make this shape id the current one.
        Any openGL primitive will then be drawn into the HL buffer
        Should be called just before any drawable is drawn (but after any modelview/projection matrix is
        modified).
        */
        virtual void postDraw() const {}

        bool contains(const Shape *shape) const;
        bool contains(HLuint shape_id) const;

        /*!
        Ends the current hl shape
        Should be called just after any drawable is drawn.
        */
        virtual void preDraw() const {}

        void addChild(Shape *shape) { 
          m_children[shape] = shape; 
          m_children_id[shape->getShapeID()] = shape; 
        }

        bool removeChild(Shape *shape) { 
          ShapeMap::iterator it=m_children.find(shape);
          if (it != m_children.end()) {
            m_children.erase(it);
        
            ShapeIDMap::iterator idIt = m_children_id.find(shape->getShapeID());
            if (idIt != m_children_id.end()) {
              m_children_id.erase(idIt);
              return true;
            }
            return false;
          }
          return false;
        }

        unsigned int getNumChildren() const { return m_children.size(); }

        virtual void apply(osg::State& state) const {}

    protected :
      /// Destructor
      virtual ~ShapeComposite();

      ShapeMap m_children;
      ShapeIDMap m_children_id;

    };

  } // osgHaptics


#endif
