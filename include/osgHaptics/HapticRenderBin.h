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

#ifndef __HAPTIC_RENDERBIN_H__
#define __HAPTIC_RENDERBIN_H__

#include <osgHaptics/export.h>
#include <osgUtil/RenderBin>
#include <osgHaptics/HapticRenderLeaf.h>
#include <osgHaptics/Shape.h>



  namespace osgHaptics {
  
class OSGHAPTICS_EXPORT HapticRenderBin : public osgUtil::RenderBin {
public:

  HapticRenderBin();

  HapticRenderBin(SortMode mode);

  virtual void drawImplementation(osg::State& state,osgUtil::RenderLeaf*& previous);

  /** Copy constructor using CopyOp to manage deep vs shallow copy.*/
  HapticRenderBin(const HapticRenderBin& rhs,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

  virtual osg::Object* cloneType() const { return new HapticRenderBin(); }
  virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new HapticRenderBin(*this,copyop); } // note only implements a clone of type.
  virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const HapticRenderBin*>(obj)!=0L; }
  virtual const char* libraryName() const { return "osgHaptics"; }
  virtual const char* className() const { return "HapticRenderBin"; }

  bool hasBeenDrawn(osg::State& state);
  /// It the state has a shape attached, then return it
  const osgHaptics::Shape *getShape(osg::State& state) const;

protected:

  void renderHapticLeaf(osgUtil::RenderLeaf* original, osg::State& state, osgUtil::RenderLeaf *previous); 


private:
  osg::ref_ptr<HapticRenderLeaf> m_haptic_renderleaf;
  virtual ~HapticRenderBin();

  typedef std::map<const osgHaptics::Shape *, const osgHaptics::Shape *> ShapeMap;
  ShapeMap m_rendered_shapes;

  int m_last_frame;
};

  } // namespace osgHaptics

#endif
