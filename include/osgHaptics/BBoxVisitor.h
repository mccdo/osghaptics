//C++ header - vrutils


#ifndef __BBoxVisitor_h__
#define __BBoxVisitor_h__


#include <osg/NodeVisitor>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>
#include <assert.h>
#include "osgHaptics/export.h"
#include <osg/BoundingBox>


/// Extracts the bounding box for a given node
namespace osgHaptics {

  class OSGHAPTICS_EXPORT BBoxVisitor : public osg::NodeVisitor{
  public:
    BBoxVisitor();

    void init() { m_bbox.init(); }  
    /// Destructor
    virtual ~BBoxVisitor(){};

    virtual void apply(osg::Geode& node);
    virtual void apply(osg::MatrixTransform& node);
    virtual void apply(osg::PositionAttitudeTransform& node);

    void pushMatrix(const osg::Matrix& matrix);
    void popMatrix();

    inline const osg::BoundingBox& getBoundingBox() const
    {
      return m_bbox;
    }

  private:
    typedef std::vector<osg::ref_ptr<osg::RefMatrix> > MatrixStack;

    MatrixStack m_matrix_stack;
    osg::BoundingBox m_bbox;
  };
} // namespace 
#endif
