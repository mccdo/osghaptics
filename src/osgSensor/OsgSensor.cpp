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

#include <osgSensor/OsgSensor.h>
#include <osg/MatrixTransform>
#include <osg/Notify>

#include <osg/Vec4>
#include <osg/io_utils>

#ifndef _WIN32
#define FLT_EPSILON     1.192092896e-07F        /* smallest such that 1.0+FLT_EPSILON != 1.0 */
#endif


using namespace osgSensor;


OsgSensor::OsgSensor() : m_refresh_delay(0) /* In hz!! */, m_freeze_sensor(false),
  m_enable_flag(OsgSensor::EnableDefault), m_enabled(true), 
  m_matrix_propagate_flag(OsgSensor::MatrixKeep), m_sensor_id(SENSOR_NOT_SET)
{
  m_scale.set(1,1,1);
  m_last_frame = m_timer.tick();
  m_matrix = new osg::RefMatrix;
  m_calibr_matrix.makeIdentity();
}

void OsgSensor::calcMatrix() 
{ 
  // Only update every m_refresh_delay (which is 0 initially)
  if ((!m_freeze_sensor) && 
      (m_timer.delta_s(m_last_frame, m_timer.tick()) > m_refresh_delay)) {
    m_last_frame = m_timer.tick();
    assert(m_matrix.valid());

    osg::Quat calibr_q;
    calibr_q.set(m_calibr_matrix);

    m_rotation *= calibr_q;
      
    // Set rotation in matrix
    m_matrix->makeRotate(m_rotation);
    // m_rotation.get(*m_matrix); 

    // Convert local translation offset to global translation offset
    // Observe preMult (inverse of rotation matrix)
    osg::Matrix rot_mat;
    osg::Vec3 new_transl;
    rot_mat = osg::Matrix::rotate(m_rotation);
    new_transl = rot_mat.preMult(m_local_transl_offset);

    // Change translation according to both offsets
    m_translation += (m_transl_offset + new_transl);
    m_translation.set(
      m_translation[0]*m_scale[0],
      m_translation[1]*m_scale[1],
      m_translation[2]*m_scale[2]);

    m_matrix->setTrans(m_translation);


      
  }
}

OsgSensor::OsgSensor(Sensor *sensor_device, int sensor_id) : m_sensor_device(sensor_device),
    m_refresh_delay(0) /* In hz!! */, 
    m_freeze_sensor(false),
    m_enable_flag(OsgSensor::EnableDefault), 
    m_enabled(true), 
    m_matrix_propagate_flag(OsgSensor::MatrixKeep),
    m_sensor_id(sensor_id)

{
  m_scale.set(1,1,1);
  m_last_frame = m_timer.tick();
  m_matrix = new osg::RefMatrix;

  if (!m_sensor_device) 
    std::runtime_error("OsgSensor::OsgSensor(): NULL pointer as sensordevice is invalid");
}


//
const osg::Vec3& OsgSensor::getTranslation()
{
  sample();
  return m_translation;
}

//
const osg::Quat& OsgSensor::getRotation() 
{
  sample();
  return m_rotation;
}

//
const osg::Matrix& OsgSensor::getMatrix() 
{
  sample();
  return *m_matrix;
}


//
OsgSensor::~OsgSensor()
{
}

void OsgSensor::setTranslation(const osg::Vec3& translation)
{ 
 
  if (isPropagated(EnableTranslationX))
    m_translation[0] = translation[0];
  if (isPropagated(EnableNegTranslationX))
    m_translation[0] = -translation[0];
  
  if (isPropagated(EnableTranslationY))
	  m_translation[1] = translation[1];
  if (isPropagated(EnableNegTranslationY))
    m_translation[1] = -translation[1];

  if (isPropagated(EnableTranslationZ))
	  m_translation[2] = translation[2];      
  if (isPropagated(EnableNegTranslationZ))
    m_translation[2] = -translation[2];

}
    
void OsgSensor::setRotation(const osg::Quat& rotation)
{ 
  // Is this sensor enabled?
  if (!m_enabled)
    return;
    
  if (isPropagated(EnableRotation)) 
   m_rotation = rotation;
/*    else {
      float x, y, z;
      quatToEuler(rotation, x,y,z);
      if (isPropagated(EnableRotationX)) {
        osg::Quat q;
        q.makeRotate(x, 0, 0);
        m_rotation = q * m_rotation;
      }

      if (isPropagated(EnableRotationY)) {
        osg::Quat q;
        q.makeRotate(0, 0, y);
        m_rotation = q * m_rotation;
      }

      if (isPropagated(EnableRotationZ)) {
        osg::Quat q;
        q.makeRotate(z, 0, 0);
        m_rotation = q * m_rotation;
      }
    }*/
}



void OsgSensor::setRefreshRate(unsigned int hz)
{ 
  if (hz > 0) 
      m_refresh_delay = 1.0/hz; 
  else 
    osg::notify(osg::WARN) << "OsgSensor::setRefreshRate(): Invalid refreshrate (" << hz << " should be > 0" << std::endl;
}



// converts a quaternion back to eulerangles
void quatToEuler(const osg::Quat&  quat, float&h, float &p, float&b )
{
  double xs,ys,zs,wx,wy,wz,xx,xy,xz,yy,yz,zz;
  double sinP, cosP, sinB, cosB, sinH, cosH;  
  double colMatrix[4][4];

  osg::Vec4 q = quat.asVec4();

  xs = q[1] + q[1];   ys = q[2] + q[2];   zs = q[3] + q[3];
  wx = q[0] * xs;  wy = q[0] * ys;  wz = q[0] * zs;
  xx = q[1] * xs;  xy = q[1] * ys;  xz = q[1] * zs;
  yy = q[2] * ys;  yz = q[2] * zs;  zz = q[3] * zs;

  /* convert to matrix */
  colMatrix[0][0] = 1.0 - (yy + zz);
  colMatrix[0][1] = xy - wz;
  // serious error [1]
  colMatrix[0][2] = xz + wy;
  colMatrix[0][3] = 0.0;

  colMatrix[1][0] = xy + wz;
  colMatrix[1][1] = 1.0 - (xx + zz);
  colMatrix[1][2] = yz - wx;
  colMatrix[1][3] = 0.0;

  colMatrix[2][0] = xz - wy;
  colMatrix[2][1] = yz + wx;
  colMatrix[2][2] = 1.0 - (xx + yy);
  colMatrix[2][3] = 0.0;

  colMatrix[3][0] = 0.0;
  colMatrix[3][1] = 0.0;
  colMatrix[3][2] = 0.0;
  colMatrix[3][3] = 1.0;

  // from matrix to euler angles, ref see page 223 of GGEMS IV & quat.c from collide library
  sinP = -colMatrix[2][0];
  cosP = sqrt(1 - sinP*sinP);

  if ( fabs(cosP) > FLT_EPSILON ) 
    {
      sinB = colMatrix[2][1] / cosP;
      cosB = colMatrix[2][2] / cosP;
      sinH = colMatrix[1][0] / cosP;
      cosH = colMatrix[0][0] / cosP;
    } 
  else 
    {
      sinB = -colMatrix[1][2];
      cosB = colMatrix[1][1];
      sinH = 0;
      cosH = 1;
    }

  // to radians
  h = (float)(atan2(sinH, cosH));
  p = (float)(atan2(sinP, cosP));
  b = (float)(atan2(sinB, cosB));
}

void OsgSensor::sample() {

  if (!m_enabled)
    return;

  Sensor *sensor = getDevice();

  if (!sensor) {
    osg::notify(osg::WARN) << "OsgSensor::sample(): No sensordevice is available" << std::endl;
    return;
  }

  osg::Vec3  p;
  osg::Quat q;

  // Get the latest value from the sensor
  int status = 0;
  if (m_sensor_id == SENSOR_NOT_SET) 
    status = sensor->read(p, q);
  else
    status = sensor->read(m_sensor_id, p,q);
    
  if (!status) {
    std::runtime_error("OsgSensor::sample(): Error reading from sensordevice");
  }

  setTranslation(p);

  setRotation(q);

  calcMatrix();
}



