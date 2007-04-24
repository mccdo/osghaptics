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

#include "DexterityLineDrawable.h"
#include <osg/io_utils>
#include <iostream>
#include <algorithm>

bool sort_x(osg::Vec3 *p1, osg::Vec3 *p2)
{
  return p1->x() < p2->x();
}

bool sort_y(osg::Vec3 *p1, osg::Vec3 *p2)
{
  return p1->y() < p2->y();
}
bool sort_z(osg::Vec3 *p1, osg::Vec3 *p2)
{
  return p1->z() < p2->z();
}



DexterityLineDrawable::DexterityLineDrawable(CatmullRomAnimationPath *path) : 
    Drawable(), m_path(path), m_resolution(10) , m_points(0L)
{
  //setUseDisplayList(false);
   
}

void DexterityLineDrawable::setResolution(unsigned int res)
{
  m_resolution = res; 
  rebuild();
}

DexterityLineDrawable::~DexterityLineDrawable()
{
  clear();
}

void DexterityLineDrawable::clear()
{
  if (m_points)
    delete [] m_points;

  m_points = 0L;

  m_sorted_in_x.resize(0);
  m_sorted_in_y.resize(0);
  m_sorted_in_z.resize(0);
}

void DexterityLineDrawable::rebuild()
{
  clear();

  unsigned int size = m_resolution+1;
  m_points = new osg::Vec3[size];

  float delta = 1.0f/m_resolution;
  float t = 0;
  unsigned int i=0;
  for(float t = 0; t < 0.8; t+=delta, i++) {
    osg::Matrix matrix;
    m_path->getMatrix(t, matrix);
    osg::Vec3 p = matrix.getTrans();
    m_points[i] = p;
    m_sorted_in_x.push_back(&m_points[i]);
    m_sorted_in_y.push_back(&m_points[i]);
    m_sorted_in_z.push_back(&m_points[i]);
  }
  //std::cerr << "Val: " << *(m_sorted_in_x[3]) << std::endl;
  std::sort(m_sorted_in_x.begin(), m_sorted_in_x.end(), sort_x);
  std::sort(m_sorted_in_y.begin(), m_sorted_in_y.end(), sort_y);
  std::sort(m_sorted_in_z.begin(), m_sorted_in_z.end(), sort_z);
}

void DexterityLineDrawable::drawImplementation(osg::RenderInfo& state) const
{ 

  if (!m_path.valid())
    return;

  glPushAttrib(GL_ALL_ATTRIB_BITS);
  glDisable(GL_LIGHTING);
  glLineWidth(5.0);
  glColor3f(1.0, 0.2, 0.2);
  float delta = 1.0f/m_resolution;
  glBegin(GL_LINE_STRIP);
  float t = 0;
  for(float t = 0; t < 0.8; t+=delta) {
    osg::Matrix matrix;
    m_path->getMatrix(t, matrix);
    osg::Vec3 p = matrix.getTrans();
    glVertex3fv(p.ptr());
    //std::cerr << "T: " << t << " " << p << std::endl;
  }
  glEnd();

  glPopAttrib();

}

osg::BoundingBox DexterityLineDrawable::computeBound() const
{
  osg::BoundingBox bbox;
  float delta = 1.0f/m_resolution;

  for(float t = 0; t < 1.0; t+=delta) {
    t += delta;
    osg::Matrix matrix;
    m_path->getMatrix(t, matrix);
    osg::Vec3 p = matrix.getTrans();
    bbox.expandBy(p);
  }

  return bbox;
}


typedef std::vector<std::pair<double, osg::Vec3 *> > DVec;

double sort_dist_vec(DVec::value_type a, DVec::value_type b )
{
  return a.first < a.first;
}

float DexterityLineDrawable::distance(const osg::Vec3& pos, osg::Vec3& closest_point)
{
  size_t size = m_sorted_in_x.size();
  
  osg::Vec3 *p_low_x=m_sorted_in_x[0], *p_high_x=m_sorted_in_x[0];
  size_t i=1;
  // Find interval in x
  while(i < size && pos.x() > m_sorted_in_x[i]->x() ) {
    if (pos.x() > m_sorted_in_x[i]->x()) {
      p_low_x = m_sorted_in_x[i];
      if (i < size-1)
        p_high_x = m_sorted_in_x[i+1];
      else
        p_high_x = 0L;
    }
    i++;  
  }

  osg::Vec3 *p_low_y=m_sorted_in_y[0], *p_high_y=m_sorted_in_y[0];
  i=1;
  // Find interval in y
  while(i < size && pos.y() > m_sorted_in_y[i]->y() ) {
    if (pos.y() > m_sorted_in_y[i]->y()) {
      p_low_y = m_sorted_in_y[i];
      if (i < size-1)
        p_high_y = m_sorted_in_y[i+1];
      else
        p_high_y = 0L;
    }
    i++;  
  }

  osg::Vec3 *p_low_z=m_sorted_in_z[0], *p_high_z=m_sorted_in_z[0];
  i=1;
  // Find interval in y
  while(i < size && pos.z() > m_sorted_in_z[i]->z() ) {
    if (pos.z() > m_sorted_in_z[i]->z()) {
      p_low_z = m_sorted_in_z[i];
      if (i < size-1)
        p_high_z = m_sorted_in_z[i+1];
      else
        p_high_z = 0L;
    }
    i++;  
  }

  //The point with lowest square distance of low_* is closest
  double dlx,dhx,dly,dhy,dlz,dhz;
  dlx = (pos-*p_low_x).length2();
  if (p_high_x)
    dhx = (pos-*p_high_x).length2();
  dly = (pos-*p_low_y).length2();
  if (p_high_y)
    dhy = (pos-*p_high_y).length2();
  dlz = (pos-*p_low_z).length2();
  if (p_high_z)
    dhz = (pos-*p_high_z).length2();

  DVec dist_vec;
  dist_vec.push_back(std::make_pair(dlx, p_low_x));
  if (p_high_x)
    dist_vec.push_back(std::make_pair(dhx, p_high_x));
  dist_vec.push_back(std::make_pair(dly, p_low_y));
  if (p_high_y)
    dist_vec.push_back(std::make_pair(dhy, p_high_y));
  dist_vec.push_back(std::make_pair(dlz, p_low_z));
  if (p_high_z)
    dist_vec.push_back(std::make_pair(dhz, p_high_z));

  std::sort(dist_vec.begin(), dist_vec.end(), sort_dist_vec);
  
  //std::cerr << "Closest point: " << *(dist_vec[0].second) << " at distance: " << dist_vec[0].first << std::endl;

  //std::cerr << "Looked for: " << pos.x() << " Low: " << p_low_x->x() << " high: " << p_high_x->x() << std::endl;
  
  closest_point = *(dist_vec[0].second);
  return sqrt(dist_vec[0].first);
}
