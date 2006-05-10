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

#ifndef __OsgSensor_h__
#define __OsgSensor_h__

#include <osgSensor/Sensor.h>

#include <osg/Referenced>
#include <osg/Vec3>
#include <osg/Timer>
#include <osg/Quat>
#include <osg/Matrix>
#include <osg/Node>
#include <sstream>
#include <stdexcept>
#include <osg/Vec3>
#include <osgSensor/export.h>

namespace osgSensor {


  /// Class for accessing a matrix from a 6Dof device
  class OSGSENSOR_EXPORT OsgSensor : public osg::Referenced {
  public:

    /// Specifies if matrix should be inverted or not when propagated
    enum MatrixPropagateFlag {
      MatrixKeep = 0,
      MatrixInvert = 1
    };

    enum SensorID {
      SENSOR_NOT_SET=-1
    };

    /// Specifies what is propagated from the sensor to the matrix
    enum EnableFlag { 
      /// Nothing is propagated
      EnableNone=0, 

      /// Propagate translation in X
      EnableTranslationX=1, 
      /// Propagate translation in Y
      EnableTranslationY=2, 
      /// Propagate translation in Z
      EnableTranslationZ=4, 
      /// Propagate rotation in X (not in use)
      EnableRotationX=8, 
      /// Propagate rotation in Z (not in use)
      EnableRotationY=16, 
      /// Propagate rotation in Z (not in use)
      EnableRotationZ=32,

      /// Propagate negative translation in X
      EnableNegTranslationX=64, 
      /// Propagate negative translation in Y
      EnableNegTranslationY=128, 
      /// Propagate negative translation in Z
      EnableNegTranslationZ=256, 

      /// Propagate rotation in x,y,z
      EnableRotation = EnableRotationX| EnableRotationY| EnableRotationZ,
      /// Enable translation in X, Y, Z
      EnableTranslation = EnableTranslationX|EnableTranslationY|EnableTranslationZ,
      /// Enable all DOF
      EnableAll = EnableTranslation | EnableRotation,
      // Default
      EnableDefault = EnableAll
    };

    /*
      Constructor.
    */
    OsgSensor();

    /*
      Constructor.
    */
    OsgSensor(Sensor *sensor_device, int sensor_id=SENSOR_NOT_SET);

    void setDevice(Sensor *sensor) { m_sensor_device = sensor; }
    Sensor *getDevice() { return m_sensor_device.get(); }
    
    /// Reads data from the sensor and returns the position
    const osg::Vec3& getTranslation();
    
    /// Read data from the sensor and returns the rotation part (quaternion)
    const osg::Quat& getRotation();

    /// Reads data from the sensor and returns the matrix
    const osg::Matrix& getMatrix();


    /// Set the translation scale. By default (1,1,1)
    inline void setScale(const osg::Vec3& scale) { m_scale = scale; }

    /// Return the translation scale.
    inline const osg::Vec3& getScale() const { return m_scale; }

    /// Set matrix propagate flag (should matrix be inverted when propagated?)
    inline void setMatrixPropagateFlag(MatrixPropagateFlag flag) { m_matrix_propagate_flag = flag; };

    /// Get matrix propagate flag
    inline MatrixPropagateFlag getMatrixPropagateFlag() { return m_matrix_propagate_flag; };

    /// Specifies if this sensor will propagate any information or not (Same result as setPropagate(EnableNone) but it wont change the EnableFlag
    inline void setEnable(bool f) { m_enabled = f; }

    /// Reurns wether this sensor is enabled or not.
    inline bool getEnable() { return m_enabled; }

    /// Is EnableFlag f set?
    inline bool isPropagated(EnableFlag f)  {return ((m_enable_flag & f) == f);  }

    /*!
      Specifies what the sensor will propagate.
      Possibility to select what the sensor will return, position and/or orientation etc.
      See EnableFlag
    */
    inline void setPropagate(EnableFlag f) { m_enable_flag = f; }

    /*!
      Specifies what the sensor will propagate.
      Possibility to select what the sensor will return, position and/or orientation etc.
      See EnableFlag
    */
    inline EnableFlag getPropagate() { return m_enable_flag; }

    /*! 
      Take care of a keyboard event
      \param key - Pressed key
      \return true if key is handled otherwise false
    */
    virtual bool handle(int key) { return false; }

    void setRefreshRate(unsigned int hz);

    inline void setFreeze(bool flag) { m_freeze_sensor = flag; }

    inline bool getFreeze() { return m_freeze_sensor; }

    /// Set offset for sensor in global coordinate system
    inline void setTranslationOffset(const osg::Vec3& offset) { m_transl_offset = offset; };

    /// Set offset for sensor in local coordinate system
    inline void setLocalTranslationOffset(const osg::Vec3& offset) { m_local_transl_offset = offset; }

	  inline const osg::Matrix& getCalibrationMatrix() const { return m_calibr_matrix; }

    inline void setCalibrationMatrix(osg::Matrix calibr) { m_calibr_matrix = calibr; }
  protected:

    /// Destructor
    virtual ~OsgSensor();
    
    /// Reads data from the sensor and updates all internal data
    virtual void sample();

    inline void setTranslation(float x, float y, float z) { setTranslation(osg::Vec3(x,y,z));}
    inline void setRotation(float x, float y, float z, float w) { setRotation(osg::Quat(x,y,z, w));}
    
    /*! Set the position of the sensor value. Will use Enable filter to only use the non filtered values from translation 
    */
    void setTranslation(const osg::Vec3& translation);
    void setRotation(const osg::Quat& rotation); 
    void calcMatrix();


    /// Returns the name of this class
    virtual const char *className() { return "OsgSensor"; }

    EnableFlag m_enable_flag;
    bool m_enabled;

    osg::Vec3 m_translation;
    osg::Quat m_rotation;
	  osg::ref_ptr<osg::RefMatrix> m_matrix;
    int m_sensor_id;


  private:
    /// Specifies whether matrix should be inverted when propagated
    MatrixPropagateFlag m_matrix_propagate_flag;
    osg::Timer m_timer;
    osg::Timer_t m_last_frame;
    double m_refresh_delay;
    bool m_freeze_sensor;
    osg::Vec3 m_transl_offset, m_local_transl_offset;
    osg::Matrix m_calibr_matrix;
    osg::Vec3 m_scale;
    osg::ref_ptr<Sensor> m_sensor_device;
  };


  void quatToEuler(const osg::Quat&  quat, float&h, float &p, float&b );
} // namespace sensors

#endif //__OsgSensor_h__

