/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Sensor Library
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

#include <osgSensor/SensorManipulator.h>

using namespace osgSensor;

SensorManipulator::SensorManipulator() : m_link(0L)
{
  //  m_matrix = new osg::RefMatrix;
}

void SensorManipulator::link(SensorLink *link)
{
  if (!link)
    osg::notify(osg::WARN) << "SensorManipulator::link(): link should not be null." << __FILE__ <<":" <<  __LINE__ << std::endl;
  

  m_link = link;
}


void SensorManipulator::home(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us)
{
  m_matrix.makeIdentity();
  updateMatrix();

}

void SensorManipulator::updateMatrix()
{
  // Is this manipulator have a link to a sensor?
  // THen get the links matrix
  if (m_link.valid()) {
    m_matrix.set( m_link->getMatrix() );
  }

  osg::Matrix m;

  osg::Vec3 eye = osg::Vec3(0, 0, 0);
  osg::Vec3 center = osg::Vec3(0, 1, 0);
  osg::Vec3 up = osg::Vec3(0, 0, 1);


  up = (up+eye)*m_matrix;
  eye = eye*m_matrix;
  center = center*m_matrix;
  up -= eye;
  up.normalize();

  m_matrix.makeLookAt(eye, center, up);  
 
  setByInverseMatrix(m_matrix);

}

// Updates the camera with the value of the current Manipulator
bool SensorManipulator::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us)
{

  bool handled = false;

  // Unused
  //unsigned int buttonMask = ea.getButtonMask();
  switch(ea.getEventType()) {

    case(osgGA::GUIEventAdapter::FRAME):
      {
        updateMatrix();
        us.requestRedraw();
        handled = true;
      }
      break;

    case(osgGA::GUIEventAdapter::KEYDOWN):
      {
        if (m_link.valid())
          handled = m_link->handle(ea.getKey());

        if (ea.getKey()==' ') {
          home(ea,us);
          us.requestRedraw();
          us.requestContinuousUpdate(false);
          handled =  true;
        }

        updateMatrix();
        us.requestRedraw();

        break;
      }

    case(osgGA::GUIEventAdapter::RESIZE):
      {
        init(ea,us);
        updateMatrix();
        us.requestRedraw();
        handled =  true;
        break;
      }

    default:
      handled =  false;
  } // switch

  return handled;

}

void SensorManipulator::init(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us)
{
  us.requestRedraw();
}


void SensorManipulator::setByMatrix(const osg::Matrix& matrix)
{
  m_matrix = matrix; 
}

osg::Matrix SensorManipulator::getMatrix() const
{
  return m_matrix; 
}

osg::Matrix SensorManipulator::getInverseMatrix() const
{
  return osg::Matrix::inverse(m_matrix);
}


