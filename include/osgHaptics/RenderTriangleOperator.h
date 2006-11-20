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

#ifndef __RenderTriangleOperator_h__
#define __RenderTriangleOperator_h__

#include <osg/TriangleFunctor>
#include <osg/Vec3>
#include <GL/GL.h>




  namespace osgHaptics {
  

/// Class to render a geometry as pure opengl triangles or count number of triangles
class RenderTriangleOperatorBase
{

public:

  /// Constructor
  RenderTriangleOperatorBase(bool do_render=true) : m_num_vertices(0), m_do_render(do_render) { }
  void setEnableRender(bool flag) { m_do_render = flag; }
  bool getEnableRender() const { return m_do_render; }

  void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool);

protected:

  /// Returns the number of triangles processed
  unsigned int getNumberOfVertices() { return m_num_vertices; }

  /// Resets the number of triangles back to zero
  void reset() { m_num_vertices = 0; }

private:
  unsigned int m_num_vertices;
  bool m_do_render;
};

//
inline void RenderTriangleOperatorBase::operator () (const osg::Vec3& v1, 
                                                     const osg::Vec3& v2,
                                                     const osg::Vec3& v3, bool)
{
  if (!m_do_render)
    return;

  if (v1==v2 || v2==v3 || v1==v3) return;

  glBegin(GL_TRIANGLES);
  glVertex3fv(v1.ptr());
  glVertex3fv(v2.ptr());
  glVertex3fv(v3.ptr());
  glEnd();
  m_num_vertices +=3;
}


/// TriangleExtractOperator is the method inherit to create a operator that will be executed per triangle.
typedef osg::TriangleFunctor<RenderTriangleOperatorBase> RenderTriangleOperator;

} // namespace osgHaptics

#endif
