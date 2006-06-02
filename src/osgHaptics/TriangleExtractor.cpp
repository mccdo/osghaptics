// $Id: TriangleExtractor.cpp,v 1.1 2005/04/11 11:05:52 andersb Exp $

#include "osgHaptics/TriangleExtractor.h"
using namespace osgHaptics;

TriangleExtractor::TriangleExtractor(TriangleExtractOperator& op) : m_tri_op(op),
  m_num_triangles(0)
{
  setTraversalMode(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN);
}

void TriangleExtractor::extract(osg::Node& node)
{
  // If its a geode or a transformation, 
  // then extract using that node, instead of diving to far
  // without that specific information
//  osg::Geode *geode = dynamic_cast<osg::Geode *> (&node);
  //osg::MatrixTransform *transform = dynamic_cast<osg::MatrixTransform *> (&node);

  //if (geode)
    //apply(*geode);
  //else if (transform)
    //apply(*transform);
  //else
    apply(node);
}

void TriangleExtractor::pushMatrix(const osg::Matrix& matrix)
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
void TriangleExtractor::popMatrix()
{
  if (!m_matrix_stack.empty())
    {
      m_matrix_stack.pop_back();
    }
}


void TriangleExtractor::apply(osg::Node& node)
{
  traverse(node);
}



void TriangleExtractor::apply(osg::Group& node)
{
  traverse((osg::Node&)node);
}


void TriangleExtractor::apply(osg::MatrixTransform& node)
{
  osg::ref_ptr<osg::RefMatrix> matrix = new osg::RefMatrix;
  node.computeLocalToWorldMatrix(*matrix,this);

  pushMatrix(*matrix);

  traverse(node);

  popMatrix();
}

void TriangleExtractor::apply(osg::Geometry& geom)
{
  if (m_matrix_stack.size())
    m_tri_op.setMatrix(*(m_matrix_stack.back().get()));
  else
    m_tri_op.setMatrix();

  geom.accept(m_tri_op);
}


void TriangleExtractor::apply(osg::Geode& geode)
{
  for(unsigned int i = 0; i < geode.getNumDrawables(); i++ )
  {
    osg::Geometry* geom = dynamic_cast<osg::Geometry *>(geode.getDrawable(i));
    if (geom) apply(*geom);
  }
}

void TriangleExtractor::apply(osg::Switch& node)
{
    apply((osg::Group&)node);
}


void TriangleExtractor::apply(osg::LOD& node)
{
    apply((osg::Group&)node);
}


void TriangleExtractor::apply(osg::Billboard& node)
{
}
/*------------------------------------------

* $Source: p:\\colosseum3D\\Repository/colosseum3D_1.0/src/osv/TriangleExtractor.cpp,v $
* $Revision: 1.1 $ 
* $Date: 2005/04/11 11:05:52 $
* $Author: andersb $ 
* $Locker:  $

* 

	Author: Anders Backman
  VRlab, Umeå University, 2002
 
* $Log: TriangleExtractor.cpp,v $
* Revision 1.1  2005/04/11 11:05:52  andersb
* New CVS repository for Colosseum3D
*
* Revision 1.10.8.3  2004/09/28 10:30:27  andersb
* Added VelocityLimitActuator to be able to limit the linear velocity for
* an object.
*
* Revision 1.10.8.2  2004/09/28 06:01:38  andersb
* *** empty log message ***
*
* Revision 1.10.8.1  2004/09/10 13:10:43  andersbl
* *** empty log message ***
*
* Revision 1.10  2003/12/15 08:02:32  andersb
* no message
*
* Revision 1.9  2003/02/27 15:24:31  Anders Backman
* Added the concept of Actuator, a callback class running in the timeframe of the physics.
*
* Revision 1.8  2003/01/23 10:33:14  deepone
* Refmatrix
*
* Revision 1.7  2002/12/19 10:18:47  andersb
* Removed osgNew
* Now reads a osg-model file for parsing a triangle mesh
*
* Revision 1.6  2002/11/22 16:08:11  Anders Backman
* Added shadows for rendering and TriangleMesh for collision.
*
* Revision 1.5  2002/11/01 15:35:18  andersb
* no message
*
* Revision 1.4  2002/10/21 15:00:09  andersb
* Working on ExtractDynamicObjects
*
* Revision 1.3  2002/09/04 11:28:50  andersb
* New major revision.
* Object structure totally rewritten.
* Creating objects is cleaner.
* Better performance due to minimizing calls to mutex locks during simulation update.
*
--------------------------------------------*/
