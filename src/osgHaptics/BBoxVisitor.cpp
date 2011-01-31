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


#include <osg/Geode>
#include <osg/Geometry>
#include <iostream>

#include "osgHaptics/BBoxVisitor.h"
#include <vrutils/math.h>
using namespace osgHaptics;

BBoxVisitor::BBoxVisitor() : NodeVisitor()
{
  setTraversalMode(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
  init();
}


void BBoxVisitor::pushMatrix(const osg::Matrix& matrix)
{

  osg::ref_ptr<osg::RefMatrix> new_matrix;

  // Is there any matrices in the stack before?
  if (m_matrix_stack.size()) {

    // YEs, then accumulate the new matrix with the previous one.
    new_matrix = new osg::RefMatrix;
    new_matrix->mult(matrix, *(m_matrix_stack.back()));//*old_matrix);
  } else {

    // Otherwise, just store this matrix
    new_matrix = new osg::RefMatrix(matrix);
  }

  m_matrix_stack.push_back(new_matrix);
}

// Leaving the subgraph, pop the matrix from the stack
void BBoxVisitor::popMatrix()
{
  if (!m_matrix_stack.empty())
  {
    m_matrix_stack.pop_back();
  }
}







void BBoxVisitor::apply(osg::MatrixTransform& node)
{
  osg::ref_ptr<osg::RefMatrix> matrix = new osg::RefMatrix;
  node.computeLocalToWorldMatrix(*matrix,this);

  pushMatrix(*matrix);

  traverse(node);

  popMatrix();
}


void BBoxVisitor::apply(osg::PositionAttitudeTransform& node)
{
  osg::ref_ptr<osg::RefMatrix> matrix = new osg::RefMatrix;
  node.computeLocalToWorldMatrix(*matrix,this);

  pushMatrix(*matrix);

  traverse(node);

  popMatrix();
}


void BBoxVisitor::apply(osg::Geode& geode)
{

  osg::BoundingBox bbox;
  bbox = geode.getBoundingBox();

  // If there is a matrix, use the scale and scale the bounding box
  if (m_matrix_stack.size()) {
    osg::Vec3 scale = (*(m_matrix_stack.back().get())).getScale();


    osg::Vec3 mn(bbox.xMin()*scale[0], bbox.yMin()*scale[1], bbox.zMin()*scale[2]);
    osg::Vec3 mx(bbox.xMax()*scale[0], bbox.yMax()*scale[1], bbox.zMax()*scale[2]);

    osg::Vec3 min,max;
    for(int i=0;i < 3; i++) {
      min [i] = vrutils::min(mn[i], mx[i]);
      max [i] = vrutils::max(mn[i], mx[i]);
    }
    bbox.set(min,max);
  }

  m_bbox.expandBy(bbox);
}
