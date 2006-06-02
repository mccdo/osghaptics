#ifndef __osgHaptics_HashedGridDrawable_h__
#define __osgHaptics_HashedGridDrawable_h__

#include <osg/Drawable>
#include <osg/Vec3>
#include <osg/observer_ptr>
#include "osgHaptics/HashedGrid.h"
#include "osgHaptics/HapticDevice.h"
#include <osgHaptics/export.h>
  
namespace osgHaptics {



/// A class that will store a hashgrid with triangles.

/*!
  This class will store a HashGrid of Triangles.
  For each draw, a proximity test will be done with the current position of 
  the haptic proxy.
  Only the triangles within the proximity of the proxy will be rendered using pure Immediate OpenGL
*/
class OSGHAPTICS_EXPORT HashedGridDrawable : public osg::Drawable {
public:


  struct Triangle : osg::Referenced {
    Triangle(const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3): m_p1(p1), m_p2(p2), m_p3(p3) {}
    osg::Vec3 m_p1, m_p2, m_p3;
  };

  typedef HashedGrid<Triangle*, osg::ref_ptr<Triangle> > TriangleHashGrid;

public:

  /// Basic constructor
  HashedGridDrawable(TriangleHashGrid *grid=0L, HapticDevice *device=0L);

  HashedGridDrawable(const HashedGridDrawable& drawable, 
    const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) : osg::Drawable(drawable, copyop) {};

  META_Object(osg,HashedGridDrawable);

  /// Set the HashGrid ot triangles that will be used for intersection test and rendering
  void setHashGrid(TriangleHashGrid *grid) { m_hashed_grid = grid; dirtyBound(); }
    
  void drawImplementation(osg::State& state) const;

  osg::BoundingBox computeBound() const;
protected:

  virtual ~HashedGridDrawable();

private:
  osg::ref_ptr<TriangleHashGrid> m_hashed_grid;

  osg::observer_ptr<HapticDevice> m_haptic_device;
};

} // namespace

#endif
