#include <osg/Geode>
#include <osg/Geometry>
#include <iostream>

#include "osgHaptics/BBoxVisitor.h"
#include <vrutils/Math.h>
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
