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

#ifndef __SensorManipulator_h__
#define __SensorManipulator_h__

#include <map>
#include <osgSensor/SensorLink.h>
#include <osgSensor/export.h>

#include <osgGA/MatrixManipulator>


namespace osgSensor {

  /// Class for connecting Manipulators to the camera in osg
  class  OSGSENSOR_EXPORT SensorManipulator : public osgGA::MatrixManipulator {

  public:

    /// Constructor
    SensorManipulator();


    /// Set the current SensorLink
    virtual void link(SensorLink *manip);

    /// Set the current SensorLink
    virtual SensorLink *link() {return m_link.get(); }

    /** Move the camera to the default position. 
    May be ignored by manipulators if home functionality is not appropriate.*/
    virtual void home(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

    /** Start/restart the manipulator.*/
    virtual void init(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);

    /** handle events, return true if handled, false otherwise.*/
    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& us);


    /** set the position of the matrix manipulator using a 4x4 Matrix.*/
    virtual void setByMatrix(const osg::Matrix& matrix);

    /** set the position of the matrix manipulator using a 4x4 Matrix.*/
    virtual void setByInverseMatrix(const osg::Matrix& matrix) { setByMatrix(osg::Matrix::inverse(matrix)); }

    /** get the position of the manipulator as 4x4 Matrix.*/
    virtual osg::Matrix getMatrix() const;

    /** get the position of the manipulator as a inverse matrix of the manipulator, typically used as a model view matrix.*/
    virtual osg::Matrix getInverseMatrix() const;

  protected:

	  /// Destructor
	  virtual ~SensorManipulator() {};


    void updateMatrix();

    osg::ref_ptr<SensorLink> m_link;
    osg::Matrix m_matrix;

  private:
  
  };



} // namespace sensors
#endif

