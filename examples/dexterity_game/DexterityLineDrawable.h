/* -*-c++-*- OpenSceneGraph Haptics Library - * Copyright (C) 2006 VRlab, Umeå University
*
* This application is open source and may be redistributed and/or modified   
* freely and without restriction, both in commericial and non commericial applications,
* as long as this copyright notice is maintained.
* 
* This application is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#ifndef _DEXTERITY_LINE_
#define _DEXTERITY_LINE_

#include "CatmullRomAnimationPath.h"
#include <osg/Drawable>


/// Class that render a Catmull-rom spline. As well as reporting closest point
class DexterityLineDrawable : public osg::Drawable{
public:
  DexterityLineDrawable() {}
  DexterityLineDrawable(CatmullRomAnimationPath *path);

  void setResolution(unsigned int res);

  DexterityLineDrawable(const DexterityLineDrawable& drawable, 
    const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) : 
  osg::Drawable(drawable, copyop) {};

  META_Object(osg,DexterityLineDrawable)

  /// Return the distance to the nearest part of the spline
  float distance(const osg::Vec3& pos, osg::Vec3& closest_point);

  /// Render the line
  void drawImplementation(osg::State& state) const;

  osg::BoundingBox computeBound() const;

  /// Clear all allocated memoryu
  void clear();
  void rebuild();

private:

  virtual ~DexterityLineDrawable();

  typedef std::pair<osg::Vec3, osg::Vec3> Line;
  osg::Vec3 *m_points;
  
  std::vector<osg::Vec3 *> m_sorted_in_x;;
  std::vector<osg::Vec3 *> m_sorted_in_y;
  std::vector<osg::Vec3 *> m_sorted_in_z;

  unsigned int m_resolution;
  osg::ref_ptr<CatmullRomAnimationPath> m_path;
};



/* Simple class for drawing the forcevector */
class VectorDrawable : public osg::Drawable{
public:
  VectorDrawable() : Drawable() {
    setUseDisplayList(false);
    setColor(1.0f,0.0f,0.0f);
    set(osg::Vec3(0,0,0), osg::Vec3(1,1,1));
  }

  VectorDrawable(const VectorDrawable& drawable, 
    const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) : 
  osg::Drawable(drawable, copyop) {};

  META_Object(osg,VectorDrawable)


    void set(const osg::Vec3& start, const osg::Vec3& end) {
      m_start = start; m_end = end;
    }

    void setColor(float r, float g, float b ) { m_color.set(r,g,b); }

    void drawImplementation(osg::State& state) const
    {
      glPushAttrib(GL_ALL_ATTRIB_BITS);
      glDisable(GL_LIGHTING);
      glColor3fv( m_color.ptr() );

      glBegin(GL_LINES);
      glVertex3fv(m_start.ptr());
      glVertex3fv(m_end.ptr());  
      glEnd();

      glPopAttrib();
    }

    osg::BoundingBox computeBound() const 
    {
      osg::BoundingBox bbox;
      bbox.expandBy(osg::Vec3(-10,-10,-10));
      bbox.expandBy(osg::Vec3(10,10,10));
      return bbox;
    }
private:
  osg::Vec3 m_color;
  osg::Vec3 m_start, m_end;
};


#endif
