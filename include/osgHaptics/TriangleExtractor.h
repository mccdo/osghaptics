#ifndef __osgHaptics_TriangleExtractor_h__
#define __osgHaptics_TriangleExtractor_h__


#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Matrix>
#include <osg/Geometry>
#include <osg/Referenced>
#include <vector>
#include <osg/TriangleFunctor>
#include <osgHaptics/export.h>

namespace osgHaptics {

  class TriangleExtractor;

  ///This class is a base class that implements a functor, with a method triangle that is executed for each triangle visited

  /*!
  This class is used as a functor class, with its method triangle executed per triangle.
  Each triangles vertices is supplied by the TriangleExtractor class. THe vertices will be in
  world coordinates. That is all the transformations will be accumulated during the traversal.
  */
  class  OSGHAPTICS_EXPORT TriangleExtractOperatorBase : public osg::Referenced
  {

  public:
    friend class TriangleExtractor;

    /// Constructor
    TriangleExtractOperatorBase() : m_num_triangles(0) { }

    void operator () (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool);

  protected:


    /// Stores the accumulated matrix
    void setMatrix(const osg::Matrix& matrix) { m_matrix = matrix; }
    void setMatrix() { m_matrix.identity(); }

    /// Returns the number of triangles processed
    unsigned int numTriangles() { return m_num_triangles; }

    /// Resets the number of triangles back to zero
    void reset() { m_num_triangles = 0; }

    /*!
    This method is a pure virtual method that has to be inherited. 
    This method will be called for each triangle. The vertices are given in
    world coordinates using the accumulated matrix.
    */
    virtual void triangle(const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3) =0;


  private:
    unsigned int m_num_triangles;
    osg::Matrix m_matrix;
  };

  //
  inline void TriangleExtractOperatorBase::operator () (const osg::Vec3& v1, 
    const osg::Vec3& v2,
    const osg::Vec3& v3, bool)
  {
    if (v1==v2 || v2==v3 || v1==v3) return;

    // Calculate the vertices world coordinates.
    osg::Vec3 v1n, v2n ,v3n;
    v1n = m_matrix.preMult(v1);
    v2n = m_matrix.preMult(v2);
    v3n = m_matrix.preMult(v3);

    // Execute the inherited method to process the vertices
    triangle(v1n, v2n, v3n);
    m_num_triangles++;
  }


  /// TriangleExtractOperator is the method inherit to create a operator that will be executed per triangle.
  typedef osg::TriangleFunctor<TriangleExtractOperatorBase> TriangleExtractOperator;


  /// This class traverses a subgraph, accumulates transformations and excute a functor (TriangleExtractOperator per triangle.
  /*!

  The functor which is a descendent from TriangleExtractOperator has a method triangle, which will be executed per triangle.
  The triangle will have the coordinates in world coordinates, that is the matrix which is accumulated during the 
  path through the scenegraph to the triangle.
  */
  class OSGHAPTICS_EXPORT TriangleExtractor : public osg::NodeVisitor
  {
  public:

    /*!
    Constructor.
    \param op - Is the functor operator which triangle method will be executed for each triangle found during traversal. 
    */
    TriangleExtractor(TriangleExtractOperator &op);

    /// Destructor
    virtual ~TriangleExtractor(){};

    /*! Begins the triangle extraction on the subgraph node.
    \param node - The subgraph from which the triangles will be extracted.
    */
    void extract(osg::Node& node);

    /*! Begins the triangle extraction on the subgraph node which is a geode
    \param node - The subgraph from which the triangles will be extracted.
    */
    void extract(osg::Geode& node) { apply(node); }

    /*! Begins the triangle extraction on the subgraph node.
    \param node - The subgraph from which the triangles will be extracted.
    */
    void extract(osg::Transform& node) { apply(node); }
  protected:

    virtual void apply(osg::Node&);
    virtual void apply(osg::Geode& node);
    virtual void apply(osg::Billboard& node);

    virtual void apply(osg::Group& node);
    virtual void apply(osg::Transform& node);
    virtual void apply(osg::Switch& node);
    virtual void apply(osg::LOD& node);

    void apply(osg::Geometry& geom);

    void pushMatrix(const osg::Matrix& matrix);
    void popMatrix();



    TriangleExtractOperator & m_tri_op;

  private:
    unsigned int m_num_triangles;
    typedef std::vector<osg::ref_ptr<osg::RefMatrix> > MatrixStack;

    MatrixStack m_matrix_stack;
  };



}; // namespace
#endif
