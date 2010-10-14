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


#include <osgHaptics/TouchModel.h>
using namespace osgHaptics;

TouchModel::TouchModel(Mode m) : m_mode(m), m_snap_distance(0.01)
{

}
void TouchModel::apply(osg::State& state) const
{
  //hlPushAttrib(HL_TOUCH_BIT);
  switch(m_mode) {
    case(CONTACT):
      hlTouchModel(HL_CONTACT);
      break;
    case(CONSTRAINT):
      hlTouchModel(HL_CONSTRAINT);
      hlTouchModelf(HL_SNAP_DISTANCE, getSnapDistance());
      break;
  }
}

float TouchModel::calcForceToSnapDistance(float force)
{
  double kStiffness;
  hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &kStiffness);

  float d = force/kStiffness;
  return d;
}
