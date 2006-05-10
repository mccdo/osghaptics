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


#ifndef __osgHaptics_Device_h__
#define __osgHaptics_Device_h__

#include <osgSensor/Sensor.h>

#include <vector>
#include <iostream>
#include <map>
#include <fstream>
#include <queue>


#include <HL/hl.h>
#include <HDU/hduError.h>
#include <osg/Timer>

#include <osg/Referenced>
#include <osg/Matrix>
#include <osg/Vec3>
#include <OpenThreads/ScopedLock>

#include <osgHaptics/export.h>
#include <osgHaptics/ForceEffect.h>
#include <osgHaptics/ContactEventHandler.h>
#include <osgHaptics/ForceOperator.h>
#include <osgHaptics/EventHandler.h>





  namespace osgHaptics {


/// Implements a Sensor that reads data from a haptic device using OpenHaptics API 

/*!
*/
class  OSGHAPTICS_EXPORT HapticDevice : public osgSensor::Sensor
{
public:

  HapticDevice(HDstring pConfigName=HD_DEFAULT_DEVICE);

  enum InterpolationMode { 
    NO_INTERPOLATION, 
    LINEAR_INTERPOLATION, 
    CUBICAL_INTERPOLATION, 
    FILTER_INTERPOLATION 
  };

  enum DeviceModel {
    OMNI_DEVICE,
    DESKTOP_DEVICE,
    NONE_DEVICE
  };

  enum WorkspaceModel {
    VIEW_WORKSPACE,          /// Use the OpenGL Modelview space for calculating transformations and forces
    ABSOLUTE_WORKSPACE            /// Use an absolute coordinate system, independent of OpenGL view
  };

  /*!
    Return the current workspace model of the device.
    By using it in ABSOLUTE mode, all forces are transformed into absolute world space, using the
    haptic device as an absolute 6DOF device.

    In VIEWSPACE model it uses the OpenGL ModelView as a reference.
  */
  WorkspaceModel getWorkspaceModel() { return m_workspace_model; }
  
  
  void setWorkspaceModel(WorkspaceModel m) { m_workspace_model = m; }

   /// Return the type of the initialized haptic device
  DeviceModel getDeviceModel() { return m_device_model; }

  /// Virtual methods inherited from Sensor class
  
  /// Read data from the device, update internal state
  virtual void update();
  
  /// Return the number of sensors available
  virtual unsigned int getNumberOfSensors() { return 1; }

  /// Shutdown the device
  virtual void shutdown();

  /// Returns the current sensor position and orientation
  virtual int read(osg::Vec3& p, osg::Quat& q);


  void setInterpolationMode(InterpolationMode mode);
  inline InterpolationMode getInterpolationMode( void ) const { return m_interpolation_mode; }

  /// returns the name of this class
  virtual const char *className() { return "HapticDevice"; }


  struct DeviceState {
    osg::Vec3d force;
    osg::Vec3d torque;
    int update_rate;
    osg::Vec3d velocity;
    osg::Vec3d angular_velocity;
    osg::Matrix transformation;
    bool buttons[EventHandler::NUM_BUTTONS];
  };


  void beginFrame();
  void endFrame();

  void makeCurrent();

  void setProxyDamping(double damping) { m_proxy_damping = damping; }
  void setProxyStiffness(double stiffness) { m_proxy_stiffness = stiffness; }

  void setProxyPosition(const osg::Vec3d& pos);

  /*!
    Get the current angular velocity of the device
  */

  osg::Vec3 getProxyPosition() const; 

  /*!
    Get the current linear velocity of the device in worldspace
  */
  osg::Vec3 getLinearVelocity() const; 

  /*!
    Get the current angular velocity of the device
  */
  osg::Vec3 getAngularVelocity() const; 
  
  /*!
    Get the current force rendered by the haptic device in worldspace
  */
  osg::Vec3 getForce() const; 


  /*!
    Get the current torque rendered by the haptic device in worldspace
  */
  osg::Vec3 getTorque() const; 


  osg::Quat getProxyOrientation() const; 

  osg::Matrix getProxyTransform() const;

  void updateWorkspace(unsigned int width, unsigned int height, bool forced=false);
  float getCursorScale() const { return m_cursor_scale; }

  void setEnableForceOutput(bool enabled);
  bool getEnableForceOutput() const { return m_force_output_enabled; }

  void setEnableForceClamping(bool enabled);
  bool getEnableForceClamping() const { return m_force_clamping_enabled; }

  void getMaxWorkspace(osg::Vec3& min, osg::Vec3& max);

  class RenderForce {
  public:
    RenderForce() : m_time_stamp(0) {}
    RenderForce(const osg::Vec3& f, double t) : m_force(f), m_time_stamp(t) {}
    void set(const osg::Vec3& f, double t) { m_force = f; m_time_stamp = t; }
    double getTimeStamp() const { return m_time_stamp; }    
    const osg::Vec3& getForce() const { return m_force; }
    osg::Vec3 evaluate(const RenderForce& force, const double& time, bool cubical) const;

  private:
    osg::Vec3 m_force;    
    double m_time_stamp;
  };

  /*!
    Initialize the haptic device.
    Should be done as the first thing after the constructor to make sure we have
    a valid haptic context.
  */
  void init(HDstring pConfigName=HD_DEFAULT_DEVICE);

  void setRenderForce(const osg::Vec3& force);
  void getRenderForce(RenderForce &rforce ) const;
  void setTouchToWorldMatrix(const osg::Matrix& m) { m_touch_to_world_matrix = m; }
  void setPositionScale(const osg::Vec3& scale) { m_position_scale = scale; }
  void setPositionOffset(const osg::Vec3& offset) { m_position_offset = offset; }

  /// Calculates scale and offset
  void fitWorkspace(const osg::Vec3& min, const osg::Vec3& max);

  /*!
    Get the state of a specific button
    \param b - Specifices which button we are interested in.
    \return true when button is down.
  */
  bool getButtonState(EventHandler::Button b) const { return m_current_state.buttons[b]; }

  /// Add a callback object to the list of registrated event handlers
  void registerEventHandler(EventHandler *bev);

  /// Remove a callback object from the list of registrated event handlers
  void unRegisterEventHandler(EventHandler *bev);

  /// Store a force effect so its not automatically deallocated
  void registerForceEffect(ForceEffect *fe); 
  bool unRegisterForceEffect(ForceEffect *fe);

  /// Remove contact eventhandler
  void unRegisterContactEventHandler(ContactEventHandler *cb);
  
  /// Remove a contact eventhandler associated with shape cb
  bool unRegisterContactEventHandler(Shape *cb);

  /*!
    Register a ContactEventHandler so that it can recieve contact information between the proxy and the shapes
    in the scene.

    \param shape - When the proxy touches this shape, the contact event handler will be triggered
    \param cb - A pointer to the ContactEventHandler
    \param event_type - What kind of Event type will cause the triggering of the cb?
  */
  void registerContactEventHandler(Shape *shape, ContactEventHandler *cb, ContactState::ContactEvent event_type);

  /*!
    Add a ForceOperator to the list of active ForceOperators so that it will be processed.
    \param fo - A pointer to the forceoperator that will be processed.
  */
  void addForceOperator(ForceOperator *fo);

  /*!
    Remove a ForceOperator. After this call the ForceOperator will not be activated.
    \param fo - A pointer to the ForceOperator to be removed
  */
  void removeForceOperator(ForceOperator *fo);


  /// 
  void setWorldToWorkSpaceMatrix(osg::Matrix& m) { 
    OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_world_to_workspace_matrix_mutex);
    m_world_to_workspace_matrix = m; 
    m_valid_world_to_workspace_matrix = true;
  }

  /// 
  bool getWorldToWorkSpaceMatrix(osg::Matrix& m) const { 
    OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_world_to_workspace_matrix_mutex);
    bool b = m_valid_world_to_workspace_matrix;
    m = m_world_to_workspace_matrix; 
    return b;
  }


  /*!
    Specify wether shapes should be enabled for haptic rendering or not
    \param flag - If true shapes will be rendered haptically
  */
  void setEnableShapeRender(bool flag) { m_enable_shape_render = flag; }
  bool getEnableShapeRender() const { return m_enable_shape_render; }

  /// Return the square of maximum force the device can deliver
  double getMaxForce2() const { return m_max_force*m_max_force; }


  /*!
    Set the min and max extents of a haptic workspace we want to work in.
  */
  void setWorkspace(const osg::Vec3& min, osg::Vec3& max);

  enum WorkspaceMode {
    VIEW_MODE, // Use the whole viewspace as a haptic workspace
    BBOX_MODE   // Use a defined BBOX as a haptic workspace
  };

  /*! 
    Specify which Workspace mode that we want to work in.
    When working in the VIEW_MODE the whole viewfrustum will be fitted into
    the HapticWorkingVolume.
    This means when the camera moves, so does the HapticWorkingVolume.

    When Working in the BBOX_MODE the hapticWorkingVolume will be fit into a specified
    BBOX (min,max) and will be fixed in worldspace.
    So when the camera (view) moves, the haptic proxy will be stationary
    
  */
  void setWorkspaceMode(WorkspaceMode mode) { m_workspace_mode = mode; }

  /// Return the current workspace mode we are working in.
  WorkspaceMode getWorkspaceMode() const  { return m_workspace_mode; }


  /*!
    Set the TouchWorkspaceMatrix.
    This matrix will make it possible to do fine grain trimming of the HapticWorkspace.
    It can be used to make it larger (set scale to (x,y,z) where x,y,z should be < 1
    It can be used for rotation and also to translate the haptic workspace.

    OBSERVE that it also affects the data retrieved from getForce() getTorque getLinearVelocity().
    getAngularVelocity(), getProxyPosition(), etc...

  */
  void setTouchWorkspaceMatrix(const osg::Matrixd& m) { m_touch_workspace_matrix = m; }
  const osg::Matrixd& getTouchWorkspaceMatrix() const { return m_touch_workspace_matrix; }

protected:

  WorkspaceMode m_workspace_mode;
  osg::Matrixd m_touch_workspace_matrix;

  class WorkspaceStruct{
  public:
    WorkspaceStruct() : m_is_set(false) {}
    inline bool isSet() { return m_is_set; }
    
    void set(const osg::Vec3& min, osg::Vec3& max) { m_is_set = true; m_min = min; m_max = max; }
    const osg::Vec3d& getMin() const { return m_min; }
    const osg::Vec3d& getMax() const { return m_max; }
  private:
    osg::Vec3d m_min, m_max;
    bool m_is_set;
  };

  WorkspaceStruct m_workspace_limits;

  static void startScheduler();
  static bool isSchedulerStarted() { return m_scheduler_started; }

  HHD getHandle() const { return m_hHDHandle; }
  HHLRC getContext() const { return m_hHLRContext; }


  typedef std::map<ForceEffect *, osg::ref_ptr<ForceEffect>  > ForceEffectMap;
  ForceEffectMap m_force_effects;

  ~HapticDevice();

  ///
  void setSchedulerStatus(bool running);

  ///
  void initCallbacks();
  
  static HDCallbackCode HDCALLBACK  endFrameCB( void *data);
  
  static HDCallbackCode HDCALLBACK  beginFrameCB( void *data);
  
  static HDCallbackCode HDCALLBACK forceEffectCB( void *data );
  
  static HDCallbackCode HDCALLBACK DeviceDataCB( void *data );
  static HDCallbackCode HDCALLBACK SetDeviceStateCB( void *data );


  static void HLCALLBACK calibrationCallback(HLenum event,
                                      HLuint object,
                                      HLenum thread,
                                      HLcache *cache,
                                      void *userdata);

    static void HLCALLBACK buttonCallback( HLenum event,
                                           HLuint object,
                                           HLenum thread,
                                           HLcache *cache,
                                           void *data );

 std::ofstream& getLogStream() { return m_log_stream; }

  static double getTimeStamp();

  enum FilterType { BARTLETT,  HANNING, HAMMING, RECTANGULAR, BLACKMAN };
  void setFilterType(FilterType f, bool recalc=false) { m_filter_type = f; if (recalc) recalculateFilter(); }
  void setCutoff(double c, bool recalc=false) { m_cutoff = c; if (recalc) recalculateFilter(); }
  void setFilterWindowSize(unsigned int size, bool recalc=false) { m_filter_window_size = size; if (recalc)  recalculateFilter(); }

private:
  /// Each shape with a registrated callback is stored in a table. Translate from shape_id to pointer to Shape, 
  Shape *findShapeID(HLuint shape_id);

  // Contact handling
  typedef std::map<Shape *, osg::ref_ptr<ContactEventHandler> > ContactEventHandlerMap;

  //typedef std::map<HLuint, Shape *> ShapeIDMap;
  //ShapeIDMap m_shape_id_map;
  
  void scheduleForceEffectCallback(ForceEffect *);
  void unScheduleForceEffectCallback(ForceEffect *);

  typedef std::map<ForceEffect *, osg::ref_ptr<ForceEffect> > ScheduleForceEffectMap;
  ScheduleForceEffectMap m_scheduled_force_effect_callbacks;

  ContactEventHandlerMap m_contact_events;

  static void HLCALLBACK startEffectCB(HLcache *cache, void *userdata);
  static void HLCALLBACK stopEffectCB(HLcache *cache, void *userdata);
  static void HLCALLBACK computeForceCB(HLcache *cache, void *userdata);

  static void HLCALLBACK contactCallback( HLenum event,
    HLuint object,
    HLenum thread,
    HLcache *cache,
    void *data );

  static void HLCALLBACK separationCallback( HLenum event,
    HLuint object,
    HLenum thread,
    HLcache *cache,
    void *data );

  static void HLCALLBACK motionCallback( HLenum event,
    HLuint object,
    HLenum thread,
    HLcache *cache,
    void *data );


  bool m_force_output_enabled;
  bool m_force_clamping_enabled;

  float m_cursor_scale;
  HHD m_hHDHandle; // Handle
  HHLRC m_hHLRContext; // Context
  
  bool m_initialized;

  unsigned int m_width, m_height;

  static bool m_scheduler_started;

  typedef std::vector<HDCallbackCode> HDHandlerVector;
  HDHandlerVector m_hd_handles;

  RenderForce m_current_render_force, m_previous_render_force;
  
  static osg::Timer_t m_start_tick;

  DeviceState m_current_state;
  osg::Matrix m_touch_to_world_matrix;
  osg::Vec3 m_position_scale;
  osg::Vec3 m_position_offset;
  
  typedef std::map<EventHandler *, osg::ref_ptr<EventHandler> > EventHandlerMap;
  EventHandlerMap m_event_handlers;

  std::ofstream m_log_stream;
  InterpolationMode m_interpolation_mode;

  typedef std::vector<double> FilterCoefficients;
  FilterCoefficients m_filter_coefficients;


  unsigned int m_filter_window_size;
  double m_cutoff;
  FilterType m_filter_type;
  

  osg::Vec3d m_average_pos;
  unsigned int m_num_positions;

  void clearAveragePosition();
  void addAveragePosition(osg::Vec3d& pos);
  osg::Vec3d getAveragePosition();


  bool recalculateFilter();
  bool calculateFilterCoefficients( FilterCoefficients& coefficients, unsigned int M, double cutoff, enum FilterType filter_type );

  template<class T>
  T filter( std::vector<T>& in_data, FilterCoefficients& coefficients );

  friend class ForceEffect;
  std::vector<osg::Vec3> m_rendering_forces;
  void pushForceEffectOperation(ForceEffect *effect, ForceEffect::Operation op);
  bool executeForceEffectOperationQueue();
  void updateForceEffects();

  typedef std::queue<std::pair<ForceEffect::Operation, osg::ref_ptr<ForceEffect> >  >ForceEffectQueue;
  ForceEffectQueue m_force_effect_queue;

  typedef std::map<ForceOperator *, osg::ref_ptr<ForceOperator> > ForceOperatorMap;
  ForceOperatorMap m_force_operators;
  OpenThreads::Mutex m_fo_mutex;

  OpenThreads::Mutex m_modelview_mutex;
  osg::Matrix m_modelview_matrix;

  mutable OpenThreads::Mutex m_world_to_workspace_matrix_mutex;
  osg::Matrix m_world_to_workspace_matrix;
  bool m_valid_world_to_workspace_matrix;

  double m_proxy_damping, m_proxy_stiffness;
  bool m_shutting_down;
  bool m_enable_shape_render;
  double m_max_force;
  DeviceModel m_device_model;
  WorkspaceModel m_workspace_model;
};


} // namespace osgHaptics


#endif
