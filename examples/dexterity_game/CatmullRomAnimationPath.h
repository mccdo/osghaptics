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

#ifndef __CatmullRomAnimationPath_h__
#define __CatmullRomAnimationPath_h__

#include <map>
#include <istream>
#include <float.h>


#include <osg/AnimationPath>

#include <osg/Matrixf>
#include <osg/Matrixd>
#include <osg/Quat>

#include <osg/Vec3>


  /** AnimationPath encapsulates a time varying transformation pathway. Can be
  * used for updating camera position and model object position.
  * AnimationPathCallback can be attached directly to Transform nodes to
  * move subgraphs around the scene.
  */
  class  CatmullRomAnimationPath : public osg::AnimationPath
  {

  public:

    typedef std::map<double, osg::AnimationPath::ControlPoint> ControlPointMap;
    ControlPointMap m_control_map;

    CatmullRomAnimationPath();//:_loopMode(LOOP) {}

    CatmullRomAnimationPath(const CatmullRomAnimationPath& ap, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
    AnimationPath(ap,copyop),
      m_control_map(ap.m_control_map)
    {}

    META_Object(osg,CatmullRomAnimationPath);

    osg::AnimationPath::ControlPoint evaluate(double t, osg::AnimationPath::ControlPoint& A, 
      osg::AnimationPath::ControlPoint& B, 
      osg::AnimationPath::ControlPoint& C, 
      osg::AnimationPath::ControlPoint& D) const ;
  
    /** Given a specific time, return the local ControlPoint frame for a point. */
    bool getInterpolatedControlPoint(double time, osg::AnimationPath::ControlPoint &controlPoint) const;

    void setUseTangentOrientation(bool f) { m_use_tangent = f; }
    void setUpVector(const osg::Vec3& up) { m_up_vec = up; }
    const osg::Vec3& getUpVector(const osg::Vec3& up) const { return m_up_vec; }

    bool getUseTangentOrientation() const { return m_use_tangent; }

    void insert(double time,const osg::AnimationPath::ControlPoint& controlPoint);

    /** Read the animation path from a flat ASCII file stream. */
    void read(std::istream& in);

    /** Write the animation path to a flat ASCII file stream. */
    void write(std::ostream& out) const;

    double getFirstTime() const { if (!m_control_map.empty()) return m_control_map.begin()->first; else return 0.0;}
    double getLastTime() const { if (!m_control_map.empty()) return m_control_map.rbegin()->first; else return 0.0;}
    double getPeriod() const { return getLastTime()-getFirstTime();}


    bool empty() const { return m_control_map.empty(); }

    bool build ();

  protected:

    ControlPointMap::const_iterator next(ControlPointMap::const_iterator n) const;
    ControlPointMap::const_iterator prev(ControlPointMap::const_iterator n) const;

    virtual ~CatmullRomAnimationPath();
    bool m_use_tangent;
    osg::Vec3 m_up_vec;

    bool m_is_built;
  };




#endif
