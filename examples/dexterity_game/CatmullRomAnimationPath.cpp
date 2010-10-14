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

#include "CatmullRomAnimationPath.h"
#include <osg/Notify>
#include <osg/Matrix>
#include <osg/io_utils>




#include <iostream>

CatmullRomAnimationPath::CatmullRomAnimationPath(): AnimationPath(), m_use_tangent(false)
{
  m_up_vec.set(0,0,1);
}

void CatmullRomAnimationPath::insert(double time,const ControlPoint& controlPoint)
{
  m_control_map[time] = controlPoint;
}

CatmullRomAnimationPath::ControlPointMap::const_iterator CatmullRomAnimationPath::next(ControlPointMap::const_iterator n) const {
  ControlPointMap::const_iterator np1,np2,nm1;
  np1 = np2 = nm1 = m_control_map.end();


  ControlPointMap::const_iterator it = n, end = m_control_map.end();

  if (n != end)
    np1 = ++it;

  if (np1 != end)
    np2 = ++it;

  switch (getLoopMode()) {
    case(LOOP):

      if (np1==end)
        return m_control_map.begin();

      return np1;

      break;

      // In case we are swinging
    case(SWING):
    case(NO_LOOPING):
      // If we cant step forward two steps, then stop
      if (np2==end)
        return n;

      return np1;
      break;
  }
  return n;
}

CatmullRomAnimationPath::ControlPointMap::const_iterator CatmullRomAnimationPath::prev(ControlPointMap::const_iterator n) const {
  ControlPointMap::const_iterator np1,np2,nm1;
  np1 = np2 = nm1 = n;

  ControlPointMap::const_iterator it = n, end = m_control_map.end();


  switch (getLoopMode()) {
    case(LOOP):
      if (n!= m_control_map.begin())
        return --n;
      else {
        n = m_control_map.end();       
        --n;
        ////std::cerr << "helloo: "<<n->first << std::endl;
        return n;
      }
      break;

    case(NO_LOOPING):
    case(SWING):
      // Can we step back?
      if (nm1 != n)
        return nm1; // return n-1
      else 
        return n; // otherwise stop
      break;
  }
  return n;
}

bool CatmullRomAnimationPath::getInterpolatedControlPoint(double time, osg::AnimationPath::ControlPoint& controlPoint ) const
{
  if (m_control_map.empty()) {
    return false;
  }

  if (m_control_map.size() < 4) 
    return false;


  switch(_loopMode)
  {
  case(SWING):
    {
      double modulated_time = (time - getFirstTime())/(getPeriod()*2.0);
      double fraction_part = modulated_time - floor(modulated_time);
      if (fraction_part>0.5) fraction_part = 1.0-fraction_part;

      time = getFirstTime()+(fraction_part*2.0) * getPeriod();
      break;
    }
  case(LOOP):
    {
      double first = getFirstTime();
      double period = getPeriod();
      double modulated_time = (time - getFirstTime())/getPeriod();
      double fraction_part = modulated_time - floor(modulated_time);
      //std::cerr << " time: " << time;
      time = getFirstTime()+fraction_part * getPeriod();
      //std::cerr << " time: " << time << std::endl;;
      break;
    }
  case(NO_LOOPING):
    // no need to modulate the time.
    break;
  }

  ControlPoint A, B, C, D;
  ControlPointMap::const_iterator lower = m_control_map.lower_bound(time);

  double lower_time = 0, upper_time=0;
  // The lowest value is the first. 
  ControlPointMap::const_iterator it = lower;

  ControlPointMap::const_iterator upper;

  // If we are using loop then begin from the end
  it = prev(lower);

  //std::cerr << "t(" << time << " ) = " << lower->first << "  ";

  A = it->second;
  //std::cerr << it->first << ", ";   
  lower_time = it->first;


  it = next(it);

  B = it->second; //(n)
  upper_time = it->first;
  upper = it;
  //std::cerr << it->first << ", ";   

  it = next(it);
  C = (it)->second; //(n+1)
  //std::cerr << it->first << ", ";   

  it = next(it);
  D = (it)->second; //(n+2)
  //std::cerr << it->first << ", " << std::endl;   

  double dt = upper_time - lower_time;
  if (dt == 0) {
    time = 0;
    dt = 1;
  }

  double t = (time-lower_time)/dt;
  //std::cerr << "t: " << t << std::endl;
  controlPoint = evaluate((time-lower_time)/dt, A, B, C, D);


  //std::cerr << " dt: " << dt << "   " << time << " - " << lower_time << " = " << (time - lower_time)/dt << std::endl;
  // //std::cerr << controlPoint.getPosition() << std::endl;

  return true;
}


void CatmullRomAnimationPath::read(std::istream& in)
{
  while (!in.eof())
  {
    double time;
    osg::Vec3d position;
    osg::Quat rotation;
    in >> time >> position.x() >> position.y() >> position.z() >> rotation.x() >> rotation.y() >> rotation.z() >> rotation.w();
    if(!in.eof())
      insert(time,ControlPoint(position,rotation));
  }
}

#define EVALUATE_SPLINE(A,B,C,D,T,T2,T3) (0.5 * (  (2*B + ( -A + C) * T + (2*A - 5*B + 4*C - D) * T2 +( -A + 3*B - 3*C + D) * T3 )))

osg::AnimationPath::ControlPoint CatmullRomAnimationPath::evaluate(double t, ControlPoint &A, 
                                                                   ControlPoint &B, 
                                                                   ControlPoint &C, 
                                                                   ControlPoint &D) const
{

  double t2 = t*t;
  double t3 = t2*t;

  ControlPoint p;
  osg::Vec3 tmpvec;
  // Position
  for(int i=0; i < 3; i++) {
    tmpvec[i] = EVALUATE_SPLINE(A.getPosition()[i],B.getPosition()[i],C.getPosition()[i],D.getPosition()[i],t,t2,t3);
  }
  p.setPosition( tmpvec );

  // Scale
  for(int i=0; i < 3; i++) {
    tmpvec[i] = EVALUATE_SPLINE(A.getScale()[i],B.getScale()[i],C.getScale()[i],D.getScale()[i],t,t2,t3);
  }
  p.setScale( tmpvec );

  if (getUseTangentOrientation()) {
    double dt = 0.01;
    double time = t+dt;
    double time2 = time*time;
    double time3 = time2*time;

    osg::Vec3 p1,p2, tangent;
    for(int i=0; i < 3; i++) {
      p1[i] = EVALUATE_SPLINE(A.getPosition()[i],B.getPosition()[i],C.getPosition()[i],D.getPosition()[i],time,time2,time3);
    }

    time = t-dt;
    time2 = time*time;
    time3 = time2*time;


    for(int i=0; i < 3; i++) {
      p2[i] = EVALUATE_SPLINE(A.getPosition()[i],B.getPosition()[i],C.getPosition()[i],D.getPosition()[i],time,time2,time3);
    }
    tangent = (p1-p2);
    tangent.normalize();
    osg::Matrix m;
    m.makeLookAt(p.getPosition(), p.getPosition()+tangent, m_up_vec);
    m.invert(m);
    osg::Quat rot;
    rot.set(m);
    p.setRotation(rot);
  }
  else {
    // orientation
    osg::Quat rot = p.getRotation();
    rot.slerp(t, B.getRotation(), C.getRotation());
    p.setRotation(rot);
  }

  return p;
}


void CatmullRomAnimationPath::write(std::ostream& fout) const
{
  int prec = fout.precision();
  fout.precision(15);

  const TimeControlPointMap& tcpm = getTimeControlPointMap();
  for(TimeControlPointMap::const_iterator tcpmitr=tcpm.begin();
    tcpmitr!=tcpm.end();
    ++tcpmitr)
  {
    const ControlPoint& cp = tcpmitr->second;
    fout<<tcpmitr->first<<" "<<cp.getPosition()<<" "<<cp.getRotation()<<std::endl;
  }

  fout.precision(prec);
}

CatmullRomAnimationPath::~CatmullRomAnimationPath()
{
}

bool CatmullRomAnimationPath::build()
{
  if (m_control_map.size() < 4) {
    osg::notify(osg::WARN) << "CatmullRomAnimationPath::build: Spline must contain >= 4 control points." << std::endl;
    return false;
  }

  if (getLoopMode() == LOOP) {
    ControlPoint blend;


    ControlPointMap::iterator begin= m_control_map.begin();
    double t1 = begin->first;

    ControlPointMap::iterator end= m_control_map.end();
    end--;
    double t2 = end->first;

    // Calculate the blend between the first and the last control point
    blend.interpolate(0.5, m_control_map.begin()->second, (end)->second);
    insert(t1-1, blend);
    insert(t2+1, blend);

  }

  m_is_built = true;
  return true;
}

