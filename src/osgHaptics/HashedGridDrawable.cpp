
#include "osgHaptics/HashedGridDrawable.h"

using namespace osgHaptics;

HashedGridDrawable::HashedGridDrawable(TriangleHashGrid *grid, HapticDevice *device) : 
  Drawable(), m_haptic_device(device), m_hashed_grid(grid)
{
  setUseDisplayList(false);
}

HashedGridDrawable::~HashedGridDrawable()
{
  m_hashed_grid = 0L;
}


void HashedGridDrawable::drawImplementation(osg::State& state) const
{
  if (!m_haptic_device.valid() || !m_hashed_grid.valid())
    return;

  // Get the position of the proxydevice
  osg::Vec3 pos = m_haptic_device->getProxyPosition();

  // Get all the triangles that are in proximity to the proxy device
  TriangleHashGrid::HashVector result;
  osg::Timer_t start = osg::Timer::instance()->tick();
  bool found = m_hashed_grid->intersect(pos, result);
  osg::Timer_t stop = osg::Timer::instance()->tick();
//  std::cerr << "# " << result.size() << "  t: " << osg::Timer::instance()->delta_m(start,stop) << std::endl;
  if (!found)
    return;


  start = osg::Timer::instance()->tick();
  //std::cerr << "#: " << result.size() << std::endl;
  TriangleHashGrid::HashVector::const_iterator it = result.begin();

  // Make sure that triangles are rendered only once
  // So first add them to a set, where only unique triangles are stored.
  typedef std::set<Triangle *> TriangleSet;
  TriangleSet triangles;
  unsigned int n=0;
  for (; it != result.end(); it++) {
    TriangleHashGrid::HashedDataVector::const_iterator vit = (*it)->begin();  
  
    for(; vit != (*it)->end(); vit++) {
      triangles.insert((*vit).first);    
    }
  }

  // Now render the triangles
  TriangleSet::const_iterator tit = triangles.begin();
  glBegin(GL_TRIANGLES);
  for(; tit != triangles.end(); tit++) {

    glVertex3fv((*tit)->m_p1.ptr());
    glVertex3fv((*tit)->m_p2.ptr());
    glVertex3fv((*tit)->m_p3.ptr());
  }
  glEnd();

  n = triangles.size();
  stop = osg::Timer::instance()->tick();
  //std::cerr << "Draw #: " << n <<  "  t: " << osg::Timer::instance()->delta_m(start,stop) << std::endl;
}


osg::BoundingBox HashedGridDrawable::computeBound() const
{
  osg::BoundingBox bbox = m_hashed_grid->getBound();
  return bbox;
}
