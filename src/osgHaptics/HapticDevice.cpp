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


#include <osgHaptics/HapticDevice.h>
#include <osgHaptics/ShapeComposite.h>

#include <iostream>
#include <cassert>
#include <strstream>
#include <sstream>

#include <HLU/hlu.h>

#include <osg/Notify>
#include <osg/Node>
#include <osg/Vec3>
#include <osg/io_utils>

#include <GL/gl.h>

#include <vrutils/math.h>



using namespace osgHaptics;

// Static member initialization
osg::Timer_t HapticDevice::m_start_tick=0;




bool HapticDevice::m_scheduler_started = false;


HapticDevice::HapticDevice(HDstring pConfigName) : Sensor(), m_hHDHandle(HD_INVALID_HANDLE), m_hHLRContext(0), m_cursor_scale(10),
m_initialized(false), m_width(0), m_height(0), m_proxy_damping(0), m_proxy_stiffness(0.3), m_shutting_down(false), 
m_valid_world_to_workspace_matrix(false), m_enable_shape_render(true), m_max_force(0), m_device_model(NONE_DEVICE), m_workspace_model(VIEW_WORKSPACE),
m_workspace_mode(VIEW_MODE)


{
  m_start_tick = osg::Timer::instance()->tick();
  // Default EventHandler
  //registerEventHandler(new EventHandler);

  //m_log_stream.open("haptics.log", std::ios_base::trunc);

  //init(pConfigName);

}



std::string getHDErrorCodeString( HDerror error_code ) {
    switch ( error_code ) {
    case HD_SUCCESS: return "HD_SUCCESS";
      
      // Function errors 
    case HD_INVALID_ENUM: return "HD_INVALID_ENUM";
    case HD_INVALID_VALUE: return "HD_INVALID_VALUE";
    case HD_INVALID_OPERATION: return "HD_INVALID_OPERATION";
    case HD_INVALID_INPUT_TYPE: return "HD_INVALID_INPUT_TYPE";
    case HD_BAD_HANDLE: return "HD_BAD_HANDLE";
      
      // Force errors
    case HD_WARM_MOTORS: return "HD_WARM_MOTORS";
    case HD_EXCEEDED_MAX_FORCE: return "HD_EXCEEDED_MAX_FORCE";
    case HD_EXCEEDED_MAX_VELOCITY: return "HD_EXCEEDED_MAX_VELOCITY";
    case HD_FORCE_ERROR: return "HD_FORCE_ERROR";

      // Device errors
    case HD_DEVICE_FAULT: return "HD_DEVICE_FAULT";
    case HD_DEVICE_ALREADY_INITIATED: return "HD_DEVICE_ALREADY_INITIATED";
    case HD_COMM_ERROR: return "HD_COMM_ERROR";
    case HD_COMM_CONFIG_ERROR: return "HD_COMM_CONFIG_ERROR";
    case HD_TIMER_ERROR: return "HD_TIMER_ERROR";
      
      // Haptic rendering context
    case HD_ILLEGAL_BEGIN: return "HD_ILLEGAL_BEGIN";
    case HD_ILLEGAL_END: return "HD_ILLEGAL_END";
    case HD_FRAME_ERROR: return "HD_FRAME_ERROR";
      
      // Scheduler errors
    case HD_INVALID_PRIORITY: return "HD_INVALID_PRIORITY";
    case HD_SCHEDULER_FULL: return "HD_SCHEDULER_FULL";

      // Licensing errors
#ifdef _WIN32
    case HD_INVALID_LICENSE: return "HD_INVALID_LICENSE";
#endif
      
    default: return "Unknown HD Error Code";
    }
  }

  std::string getHLErrorString( HLerror error ) {
    if( error.errorCode == HL_DEVICE_ERROR ) {
      std::stringstream s;
      s << "HL_DEVICE_ERROR( " 
        << getHDErrorCodeString( error.errorInfo.errorCode )
        << " )" << std::ends;
      return s.str();
    } else {
      return error.errorCode;
    }
  }


void HapticDevice::setWorkspace(const osg::Vec3& min, osg::Vec3& max)
{
  m_workspace_limits.set(min,max);
}


/*******************************************************************************
 Use the current OpenGL viewing transforms to initialize a transform for the
 haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void HapticDevice::updateWorkspace(unsigned int width, unsigned int height, bool forced)
{

  //if (width == m_width && height == m_height && !forced)
  //  return;

  m_width = width;
  m_height = height;

//  makeCurrent();

  GLdouble modelview[16];
  GLdouble projection[16];
  GLint viewport[4];

  hlMatrixMode(HL_TOUCHWORKSPACE);
  hlLoadIdentity();

  // Use the specified touchworkspace matrix
  hlMultMatrixd(m_touch_workspace_matrix.ptr());

  if (getWorkspaceModel() == VIEW_WORKSPACE) {
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);


    
    // Should we fit the haptic workspace to a specified bbox?
    if (getWorkspaceMode() == BBOX_MODE) {
      if (!m_workspace_limits.isSet()) {
        osg::notify(osg::WARN) << "HapticDevice::updateWorkspace(): Current WorkspaceMode is BBOX_MODE, but the extents of the workspace is not defined" << std::endl;
        return;
      }
      hluFitWorkspaceBox(modelview, m_workspace_limits.getMin().ptr(), m_workspace_limits.getMax().ptr());
    }

    // Or should we fit haptic workspace to the whole view volume.
    else {
      hluFitWorkspace(projection);
    }
    osg::Matrix vt, tw;
    hlGetDoublev(HL_VIEWTOUCH_MATRIX, vt.ptr());
    hlGetDoublev(HL_TOUCHWORKSPACE_MATRIX, tw.ptr());

    //p*mv*tw

    // Matrix to move from world to haptic Workspace
    setWorldToWorkSpaceMatrix(m_modelview_matrix*vt*tw);
  }

  //std::cerr << "VT: " << vt << std::endl;
  //std::cerr << "WT: " << tw << std::endl;


  // Compute cursor scale.
  m_cursor_scale = hluScreenToModelScale(modelview, projection, viewport);
#define CURSOR_SIZE_PIXELS 20
  m_cursor_scale *= CURSOR_SIZE_PIXELS;
}



/*******************************************************************************
 This handler will get called when the application is exiting.
 Deallocates any state and cleans up.
*******************************************************************************/
HapticDevice::~HapticDevice()
{
  shutdown();
}


void HapticDevice::registerForceEffect(ForceEffect *fe)
{ 
  m_force_effects[fe] = fe; 
}

bool HapticDevice::unRegisterForceEffect(ForceEffect *fe)
{
  // If the device is shutting down, then ignore this unregister operation, otherwise we will 
  // mess up the iterators for the map holding the force effects
  if (  m_shutting_down )
    return true;

  ForceEffectMap::iterator it = m_force_effects.find(fe);
  if (it != m_force_effects.end()) {
    m_force_effects.erase(it);
    return true;
  }
  return false;
}

void HapticDevice::updateForceEffects()
{
	ForceEffectMap::iterator it = m_force_effects.begin();
	for (;it != m_force_effects.end(); it++) {
	  if (it->second->getStarted())
		it->second->executeUpdate();
	}
}


/*******************************************************************************
 Initialize the HDAPI. This involves initing a device configuration, enabling
 forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void HapticDevice::init(HDstring pConfigName)
{
  if (m_initialized) {
    osg::notify(osg::WARN) << "HapticDevice::init(): Device already initialized "<< std::endl;
    return;
  }

  setInterpolationMode(LINEAR_INTERPOLATION);


  setFilterType(HAMMING, false);
  setCutoff(0.3, false);
  setFilterWindowSize(10, false);
  recalculateFilter();

  m_position_scale.set(1,1,1);

  HDErrorInfo error;

  m_hHDHandle = hdInitDevice(pConfigName);
  if (HD_DEVICE_ERROR(error = hdGetError()))
  {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");

    std::ostrstream str;
    str << "HapticDevice::init() (" << error.errorCode << "): " << getHDErrorCodeString(error.errorCode) << std::ends;
    osg::notify(osg::WARN) << "HapticDevice::updateWorkspace(): " << str.str() << std::endl;
    throw std::runtime_error(str.str());
  }
    
  m_hHLRContext = hlCreateContext(m_hHDHandle);
  makeCurrent();

  std::string model = hdGetString(HD_DEVICE_MODEL_TYPE);
  std::string serial = hdGetString(HD_DEVICE_SERIAL_NUMBER);

  osg::notify(osg::INFO) << "Found device model: " << model << "/ serial number: " << serial
     << ".\n\n" << std::endl;    

  if (model == "PHANTOM Omni") 
    m_device_model = OMNI_DEVICE;

  // Enable optimization of the viewing parameters when rendering
  // geometry for OpenHaptics
  hlEnable(HL_HAPTIC_CAMERA_VIEW);

  // generate id's for the three shapes
  //sphereShapeId = hlGenShapes(1);

  hlTouchableFace(HL_FRONT);

  setEnableForceOutput(true);
  setEnableForceClamping(true);

  setSchedulerStatus(true);

  initCallbacks();

  // Check what the max force is
  hdGetDoublev(HD_NOMINAL_MAX_FORCE, &m_max_force);

  m_initialized = true;
}


HDCallbackCode HDCALLBACK HapticDevice::forceEffectCB( void *data ) {
  HapticDevice *device = static_cast< HapticDevice * >( data );
    
    // get current values from HD API 
  HLdouble m[16];
  hdGetDoublev( HD_CURRENT_TRANSFORM, m );
  osg::Matrix matrix(
      m[0], m[1], m[2], m[3],
      m[0], m[1], m[2], m[3],
      m[0], m[1], m[2], m[3],
      m[0], m[1], m[2], m[3]);

    hdGetDoublev( HD_CURRENT_VELOCITY, m );
    osg::Vec3 velocity( m[0],  m[1], m[2] );
    

    // add the resulting force and torque to the rendered force.
    osg::Vec3d force;
    hdGetDoublev( HD_CURRENT_FORCE,force.ptr() );

    osg::Vec3d torque;
    hdGetDoublev( HD_CURRENT_TORQUE,torque.ptr() );

    // Transform the internal Hapticforce to a force in world coordinates
    //force = device->m_touch_to_world_matrix.preMult(force);

    // Transform force and torque into World coordinates
    osg::Matrix w2w_matrix;
    bool is_valid = device->getWorldToWorkSpaceMatrix(w2w_matrix);
  
    // is_valid is only true when we have update the matrix
    // As this is done in a separate thread we have to make sure
    if (is_valid) {
      double current_time = device->getTimeStamp();
      device->m_log_stream << current_time;

      // Iterate over all ForceOperators and add the force together
      OpenThreads::ScopedLock<OpenThreads::Mutex> sl(device->m_fo_mutex);
      ForceOperatorMap::iterator it=device->m_force_operators.begin();
      for(;it != device->m_force_operators.end(); it++) {
        it->first->update();

        // Set the WorldToHapticWorkspace matrix
        it->first->setWorldToWorkSpaceMatrix(w2w_matrix);
        if (it->first->getEnable()) {
          osg::Vec3d out;
          it->first->calculateForce(force, out, current_time);
          double l = out.length2();
          if (l > device->getMaxForce2()) {
            l = sqrt(l);          
            out.normalize();
            out *= l;
            //device->warning("forceEffectCB") << "ForceOperator delivered too large force: " << out << std::endl;
          }
          force += out;
          it->first->calculateTorque(torque, out, current_time);
          torque += out;
        }
      } // for
    } // if is_valid

    double l = force.length2();
    if (l > device->getMaxForce2()) {
      l = sqrt(l);
      force *= l;
      //device->warning("forceEffectCB") << "Total force exeeds maximum: " << force << std::endl;
    }

    hdSetDoublev( HD_CURRENT_FORCE, force.ptr() );
    hdSetDoublev( HD_CURRENT_TORQUE, torque.ptr() );
    
    return HD_CALLBACK_CONTINUE;
  }

void HapticDevice::setInterpolationMode(InterpolationMode mode)
{
//  OpenThreads::ScopedLock<OpenThreads::Mutex> scope(m_mutex);
  m_interpolation_mode = mode;
}


osg::Vec3 HapticDevice::RenderForce::evaluate(const RenderForce &previous_force, const double& time, bool smooth) const
{
  osg::Vec3 interpolated_force;
  double interval = getTimeStamp() - previous_force.getTimeStamp();

  if (abs(interval) < 1E-10)
    return osg::Vec3(0,0,0);

  double t = (time - getTimeStamp())/interval;

  if (smooth) {
    
    double st = vrutils::smoothstep<double>(0,1, t);    
    //std::cerr << "A: " << getTimeStamp() << "  B: " << getTimeStamp()+interval << "  st: " << st << std::endl;
    interpolated_force = vrutils::mix(previous_force.getForce(), getForce(), st);

  } else
    interpolated_force = vrutils::mix(previous_force.getForce(), getForce(), t);
  return interpolated_force;
}

double HapticDevice::getTimeStamp()
{

  osg::Timer_t now = osg::Timer::instance()->tick();
  double time = osg::Timer::instance()->delta_s(m_start_tick, now);

  return time;
}


void HapticDevice::setRenderForce(const osg::Vec3& force)
{
//  OpenThreads::ScopedLock<OpenThreads::Mutex> scope(m_mutex);

  m_previous_render_force = m_current_render_force;
  m_current_render_force.set(force, getTimeStamp());

  //warning("size") << "Size : " << m_rendering_forces.size() << std::endl;
  m_rendering_forces.push_back(force);
  if(m_rendering_forces.size() > 10)
    m_rendering_forces.erase(m_rendering_forces.begin());

}



void HapticDevice::getRenderForce(RenderForce& rforce) const
{
//  OpenThreads::ScopedLock<OpenThreads::Mutex> scope(m_mutex);
  rforce = m_current_render_force;  
}



void HapticDevice::initCallbacks()
{
  HDCallbackCode handle =
    hdScheduleAsynchronous( HapticDevice::forceEffectCB,
                            this,
                            HD_DEFAULT_SCHEDULER_PRIORITY );
  m_hd_handles.push_back( handle );

  handle = hdScheduleAsynchronous( HapticDevice::beginFrameCB,
                                   this,
                                   HD_MAX_SCHEDULER_PRIORITY );
  m_hd_handles.push_back( handle );
  handle = hdScheduleAsynchronous( HapticDevice::endFrameCB,
                                   this,
                                   HD_MIN_SCHEDULER_PRIORITY );
  m_hd_handles.push_back( handle );


  hlAddEventCallback( HL_EVENT_1BUTTONDOWN, 
                      HL_OBJECT_ANY,
                      HL_CLIENT_THREAD,
                      HapticDevice::buttonCallback,
                      this );
  
  hlAddEventCallback( HL_EVENT_1BUTTONUP, 
                      HL_OBJECT_ANY,
                      HL_CLIENT_THREAD,
                      HapticDevice::buttonCallback,
                      this );
  
  hlAddEventCallback( HL_EVENT_2BUTTONDOWN, 
                      HL_OBJECT_ANY,
                      HL_CLIENT_THREAD,
                      HapticDevice::buttonCallback,
                      this );
  
  hlAddEventCallback( HL_EVENT_2BUTTONUP, 
                      HL_OBJECT_ANY,
                      HL_CLIENT_THREAD,
                      HapticDevice::buttonCallback,
                      this );
  
  hlAddEventCallback( HL_EVENT_CALIBRATION_UPDATE,
                      HL_OBJECT_ANY, HL_CLIENT_THREAD,
                      &HapticDevice::calibrationCallback, 
                      this);
  
  hlAddEventCallback( HL_EVENT_CALIBRATION_INPUT,
                      HL_OBJECT_ANY, HL_CLIENT_THREAD,
                      &HapticDevice::calibrationCallback, 
                      this);
  
}

/* This function will be invoked when a calibration event occurs. This
handler notifies the user to either provide calibration input or that
calibration is about to be updated. This callback will be invoked as a
result of calling hlCheckEvents() within the main loop of the program.
*/
void HLCALLBACK HapticDevice::calibrationCallback(HLenum event,
                                    HLuint object,
                                    HLenum thread,
                                    HLcache *cache,
                                    void *userdata)
{
  HapticDevice *device = static_cast< HapticDevice * >( userdata );
  EventHandlerMap::iterator it;
  for(it = device->m_event_handlers.begin(); it != device->m_event_handlers.end(); it++) 

    if (event == HL_EVENT_CALIBRATION_UPDATE)
    {
      it->first->setType(EventHandler::CALIBRATION);
      it->first->calibrate(EventHandler::UPDATE);

      hlUpdateCalibration();
    }
    else if (event == HL_EVENT_CALIBRATION_INPUT)
    {
      it->first->setType(EventHandler::CALIBRATION);
      it->first->calibrate(EventHandler::INPUT);
      //std::cout << "Device requires calibration input..." << std::endl;
    }
}


void HLCALLBACK HapticDevice::buttonCallback( HLenum event,
                                        HLuint object,
                                        HLenum thread,
                                        HLcache *cache,
                                        void *data ) 
{
  HapticDevice *device = static_cast< HapticDevice * >( data );
  EventHandlerMap::iterator it;
  for(it = device->m_event_handlers.begin(); it != device->m_event_handlers.end(); it++) 
  {  
    if( event == HL_EVENT_1BUTTONDOWN ) {
      device->m_current_state.buttons[0] = true;
      it->first->setType(EventHandler::BUTTON);
      it->first->setState(EventHandler::BUTTON_1, EventHandler::DOWN);
      it->first->push(EventHandler::BUTTON_1);
    }
    else if( event == HL_EVENT_1BUTTONUP ) {
      device->m_current_state.buttons[0] = false;
      it->first->setType(EventHandler::BUTTON);
      it->first->setState(EventHandler::BUTTON_1, EventHandler::UP);
      it->first->release(EventHandler::BUTTON_1);
    }
    else if( event ==  HL_EVENT_2BUTTONDOWN ) {
      device->m_current_state.buttons[1] = true;
      it->first->setType(EventHandler::BUTTON);
      it->first->setState(EventHandler::BUTTON_2, EventHandler::DOWN);
      it->first->push(EventHandler::BUTTON_2);
    }
    else if( event == HL_EVENT_2BUTTONUP ) {
      device->m_current_state.buttons[1] = false;
      it->first->setType(EventHandler::BUTTON);
      it->first->setState(EventHandler::BUTTON_2, EventHandler::UP);
      it->first->release(EventHandler::BUTTON_2);
    }
  } // for
}



HDCallbackCode HDCALLBACK HapticDevice::DeviceDataCB( void *data ) {
  HapticDevice::DeviceState *state = static_cast< HapticDevice::DeviceState * >( data );
  HDint asd;
  hdGetIntegerv( HD_UPDATE_RATE, &(asd) );
  hdGetIntegerv( HD_UPDATE_RATE, &(state->update_rate) );
  hdGetDoublev( HD_LAST_FORCE, state->force.ptr() );
  hdGetDoublev( HD_LAST_TORQUE, state->torque.ptr() );
  hdGetDoublev( HD_CURRENT_TRANSFORM, state->transformation.ptr());
  hdGetDoublev( HD_CURRENT_VELOCITY, state->velocity.ptr());
  hdGetDoublev( HD_CURRENT_ANGULAR_VELOCITY, state->angular_velocity.ptr());
  return HD_CALLBACK_DONE;
} 

void HapticDevice::update()
{
  hdScheduleSynchronous( HapticDevice::DeviceDataCB,
                         &m_current_state,
                         HD_DEFAULT_SCHEDULER_PRIORITY ); 

  hlCheckEvents();
  HLerror error;
  while ( HL_ERROR(error = hlGetError()) ) {
    osg::notify(osg::WARN) << getHLErrorString( error )
      << std::endl;
  }

  osg::Matrix m;
  hlGetDoublev(HL_PROXY_TRANSFORM, m_current_state.transformation.ptr());


  // Convert the transformationmatrix according the touch_to_world matrix
  /*m_current_state.transformation.postMult(m_world_to_workspace_matrix);
  osg::Vec3 t= m_current_state.transformation.getTrans();
  t.set(t[0] * m_position_scale[0], t[1] * m_position_scale[1], t[2] * m_position_scale[2]); 
  t += m_position_offset;
  m_current_state.transformation.setTrans(t);*/
}

void HapticDevice::shutdown()
{  

  if (!m_initialized)
    return;

  m_shutting_down = true;
  
  m_scheduled_force_effect_callbacks.clear();
  m_contact_events.clear();
  m_hd_handles.clear();
  m_event_handlers.clear();
  m_force_effects.clear();
  m_force_operators.clear();
  
  
  // free up the haptic rendering context
  hlMakeCurrent(NULL);
  if (m_hHLRContext != NULL)
  {
    hlDeleteContext(m_hHLRContext);
  }

  for( HDHandlerVector::iterator it = m_hd_handles.begin(); it != m_hd_handles.end();  it++ ) {
    hdUnschedule(*it);
  }

  // free up the haptic device
  if (m_hHDHandle != HD_INVALID_HANDLE)
  {
    hdDisableDevice(m_hHDHandle);
  }
  


  m_initialized = false;
  return;

}

void HapticDevice::fitWorkspace(const osg::Vec3& min, const osg::Vec3& max)
{
  osg::Vec3 wMin, wMax;
  getMaxWorkspace(wMin, wMax);

  osg::Vec3 dw, d;
  dw[0] = fabs(wMax[0] - wMin[0]);
  dw[1] = fabs(wMax[1] - wMin[1]);
  dw[2] = fabs(wMax[2] - wMin[2]);

  d[0] = fabs(max[0] - min[0]);
  d[1] = fabs(max[1] - min[1]);
  d[2] = fabs(max[2] - min[2]);

  m_position_scale[0] = d[0]/dw[0];
  m_position_scale[1] = d[1]/dw[1];
  m_position_scale[2] = d[2]/dw[2];

  m_position_offset = min;
}


HDCallbackCode HDCALLBACK  HapticDevice::beginFrameCB( void *data)
{
  assert(data);
  HapticDevice *device = static_cast< HapticDevice * >(data);
  
  HHD hHD = hdGetCurrentDevice();   
  hdBeginFrame( device->getHandle() );

  return HD_CALLBACK_CONTINUE;  
}


void HapticDevice::pushForceEffectOperation(ForceEffect *effect, ForceEffect::Operation op)
{
  m_force_effect_queue.push(std::make_pair(op, effect));
}

bool HapticDevice::executeForceEffectOperationQueue()
{
  ForceEffectQueue::value_type item;
  bool found=false;
  while(m_force_effect_queue.size()) {
    item = m_force_effect_queue.front();
    switch(item.first) {
      case(ForceEffect::START):
        item.second->executeStart();
        break;
      case(ForceEffect::STOP):
        item.second->executeStop();
        break;
      case(ForceEffect::TRIG):
        item.second->executeTrig();
        break;
      case(ForceEffect::UPDATE):
        item.second->executeUpdate();
        break;
    }
    found = true;
    m_force_effect_queue.pop();
  }
  return found;
}


HDCallbackCode HDCALLBACK  HapticDevice::endFrameCB( void *data)
{
  assert(data);
  HapticDevice *device = static_cast< HapticDevice * >(data);
  
  HHD hHD = hdGetCurrentDevice();    
  hdEndFrame( device->getHandle() );

  return HD_CALLBACK_CONTINUE;  
}


void HapticDevice::makeCurrent()
{
  hlMakeCurrent(m_hHLRContext);
}

void HapticDevice::beginFrame()
{
  
  makeCurrent();

  if (getWorkspaceModel() == VIEW_WORKSPACE) {
    OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_modelview_mutex);
    glGetDoublev(GL_MODELVIEW_MATRIX, m_modelview_matrix.ptr());
  }

  hlBeginFrame();

  HLerror error;
  while ( HL_ERROR(error = hlGetError()) ) {
    osg::notify(osg::WARN) << getHLErrorString( error )
      << std::endl;
  }      
}

void HapticDevice::endFrame()
{

  
  makeCurrent();
  
  updateForceEffects();
  executeForceEffectOperationQueue();

  hlEndFrame(); 
  
   // check for any errors
  HLerror error;
  while ( HL_ERROR(error = hlGetError()) ) {
    osg::notify(osg::WARN) << getHLErrorString( error )
      << std::endl;
  }
}

osg::Matrix HapticDevice::getProxyTransform() const 
{    
  /*osg::Matrix m;
  hlGetDoublev(HL_PROXY_TRANSFORM, m.ptr());

  return m;;
  */
	return m_current_state.transformation;
}


osg::Vec3 HapticDevice::getProxyPosition() const 
{    
/*
  osg::Matrix m;
  hlGetDoublev(HL_PROXY_TRANSFORM, m.ptr());
  return m.getTrans();
*/
  return m_current_state.transformation.getTrans();
}

osg::Vec3 HapticDevice::getLinearVelocity() const 
{    
	osg::Matrix m;
  getWorldToWorkSpaceMatrix(m);
	osg::Quat q;
	q.set(m);
	m.set(q);
	return m.postMult(m_current_state.velocity);
}

osg::Vec3 HapticDevice::getForce() const 
{    
	osg::Matrix m;
  getWorldToWorkSpaceMatrix(m);
	osg::Quat q;
	q.set(m);
	m.set(q);
	return m.postMult(m_current_state.force);
}

osg::Vec3 HapticDevice::getTorque() const 
{    
  osg::Matrix m;
  getWorldToWorkSpaceMatrix(m);
  return m.postMult(m_current_state.torque);
}

osg::Vec3 HapticDevice::getAngularVelocity() const 
{    
  osg::Matrix m;
  getWorldToWorkSpaceMatrix(m);
  return m.postMult(m_current_state.angular_velocity);
}

osg::Quat HapticDevice::getProxyOrientation() const 
{    
  osg::Quat q;
/*
  osg::Matrix m;
  hlGetDoublev(HL_PROXY_TRANSFORM, m.ptr());

  q.set(m);
*/
  q.set(m_current_state.transformation);
  return q;
}


void HapticDevice::setEnableForceOutput(bool f)
{
  makeCurrent();
  if (f) {
    hdEnable(HD_FORCE_OUTPUT);
  }
  else {
    hdDisable(HD_FORCE_OUTPUT);
  }
  m_force_output_enabled = f;
}

void HapticDevice::setEnableForceClamping(bool f)
{
  makeCurrent();
  if (f) {
    hdEnable(HD_MAX_FORCE_CLAMPING);
  }
  else {
    hdDisable(HD_MAX_FORCE_CLAMPING);
  }
  m_force_clamping_enabled = f;
}


void HapticDevice::getMaxWorkspace(osg::Vec3& min, osg::Vec3& max)
{
  makeCurrent();
  HDdouble maxWorkspace[6];
  hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);
  printf("%lf %lf %lf %lf %lf %lf\n", 
      maxWorkspace[0],
      maxWorkspace[1],
      maxWorkspace[2],
      maxWorkspace[3],
      maxWorkspace[4],
      maxWorkspace[5]);
  min.set( maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
  max.set( maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);

}

void HapticDevice::setSchedulerStatus(bool running)
{
  if (running && !m_scheduler_started ) {
    hdStartScheduler();
  } else {
    hdStopScheduler();
  }
  
  m_scheduler_started = running;
}

void HapticDevice::scheduleForceEffectCallback(ForceEffect *fe)
{
  hlBeginFrame();
  hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc)HapticDevice::computeForceCB, (void *)fe);
  hlCallback(HL_EFFECT_START,(HLcallbackProc) HapticDevice::startEffectCB,(void*)fe);
  hlCallback(HL_EFFECT_STOP, (HLcallbackProc)HapticDevice::stopEffectCB, (void*)fe);

  m_scheduled_force_effect_callbacks[fe] = fe;
}

void HLCALLBACK HapticDevice::startEffectCB(HLcache *cache, void *userdata)
{
  osg::notify(osg::INFO) << "HapticDevice::startEffectCB" << std::endl;
}

void HLCALLBACK HapticDevice::stopEffectCB(HLcache *cache, void *userdata)
{
  osg::notify(osg::INFO) << "HapticDevice::stopEffectCB" << std::endl;
}

void HLCALLBACK HapticDevice::computeForceCB(HLcache *cache, void *userdata)
{
  osg::notify(osg::INFO) << "HapticDevice::computeForceCB" << std::endl;
}


void HapticDevice::addForceOperator(ForceOperator *fo)
{
  OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_fo_mutex);
  m_force_operators[fo] = fo;
}

void HapticDevice::removeForceOperator(ForceOperator *fo)
{
  ForceOperatorMap::iterator it = m_force_operators.find(fo);
  if (it != m_force_operators.end()) {
    
    if (!m_shutting_down) {
      OpenThreads::ScopedLock<OpenThreads::Mutex> sl(m_fo_mutex);
      m_force_operators.erase(it);
    }
    else
      m_force_operators.erase(it);
  }
}


void HapticDevice::unScheduleForceEffectCallback(ForceEffect *fe)
{
  //m_scheduled_force_effect_callbacks[fe] = fe;
}


int HapticDevice::read(osg::Vec3& p, osg::Quat& q)
{
  p = getProxyPosition(); //m_current_state.transformation.getTrans();
  q = getProxyOrientation(); //.set(m_current_state.transformation);

  //osg::Matrix m;
  //hdGetDoublev(HD_CURRENT_TRANSFORM, m.ptr());

  return 1;
}


void HapticDevice::contactCallback( HLenum event,
                                       HLuint object,
                                       HLenum thread,
                                       HLcache *cache,
                                       void *data )
{
  assert(data);

  HapticDevice *device = static_cast<HapticDevice *>(data);
  
  Shape *shape = device->findShapeID(object);

  if (!shape) {
    osg::notify(osg::WARN) << "contactCallback: Unable to find matching Shape " << std::endl; 
    return;
  }

  // Now find a matching callback and execute that one
  ContactEventHandlerMap::iterator it = device->m_contact_events.find(shape);
  if (it == device->m_contact_events.end()) {
    osg::notify(osg::WARN) << ("contactCallback") << "No matching callback found for shape: " << shape->getName() << std::endl;
    return;
  }

  // We found a callback!
  ContactState state;

  osg::Vec3d p;
  hlCacheGetDoublev( cache, 
    HL_PROXY_POSITION,
    p.ptr() );

  osg::Vec3d n;
  hlCacheGetDoublev( cache, 
    HL_PROXY_TOUCH_NORMAL,
    n.ptr() );


  state.set(ContactState::Contact, shape, device, p, n, getTimeStamp());

  // Call the callback
  (*it).second->execute(state);
}

void HapticDevice::separationCallback( HLenum event,
                                       HLuint object,
                                       HLenum thread,
                                       HLcache *cache,
                                       void *data )
{
  assert(data);

  HapticDevice *device = static_cast<HapticDevice *>(data);

  Shape *shape = device->findShapeID(object);

  if (!shape) {
    osg::notify(osg::WARN) << "separationCallback: Unable to find matching Shape " << std::endl; 
    return;
  }

  // Now find a matching callback and execute that one
  ContactEventHandlerMap::iterator it = device->m_contact_events.find(shape);
  if (it == device->m_contact_events.end()) {
    osg::notify(osg::WARN) << ("separationCallback") << "No matching callback found for shape: " << shape->getName() << std::endl;
    return;
  }

  // We found a callback!
  ContactState state;


  state.set(ContactState::Separation, shape, device, getTimeStamp());
  // Execute the callback
  (*it).second->execute(state);

}

void HapticDevice::motionCallback( HLenum event,
                                         HLuint object,
                                         HLenum thread,
                                         HLcache *cache,
                                         void *data )
{
  assert(data);

  HapticDevice *device = static_cast<HapticDevice *>(data);

  Shape *shape = device->findShapeID(object);

  if (!shape) {
    osg::notify(osg::WARN) << "motionCallback: Unable to find matching Shape " << std::endl; 
    return;
  }

  // Now find a matching callback and execute that one
  ContactEventHandlerMap::iterator it = device->m_contact_events.find(shape);
  if (it == device->m_contact_events.end()) {
    osg::notify(osg::WARN) << ("motionCallback") << "No matching callback found for shape: " << shape->getName() << std::endl;
    return;
  }

  // We found a callback!
  ContactState state;

  osg::Vec3d p;
  hlCacheGetDoublev( cache, 
    HL_PROXY_POSITION,
    p.ptr() );

  osg::Vec3d n;
  hlCacheGetDoublev( cache, 
    HL_PROXY_TOUCH_NORMAL,
    n.ptr() );


  state.set(ContactState::Motion, shape, device, p, n, getTimeStamp());
  (*it).second->execute(state);
}

void HapticDevice::unRegisterContactEventHandler(ContactEventHandler *cb)
{
  // If the device is shutting down, then ignore this unregister operation, otherwise we will 
  // mess up the iterators for the map holding the contact handlers
  if (m_shutting_down)
    return;



  // remove from callback map AND shape_id_map
  ContactEventHandlerMap::iterator it = m_contact_events.begin();
  for(; it != m_contact_events.end(); it++) {
    if ((it->second).get() == cb) {

      hlRemoveEventCallback( HL_EVENT_TOUCH, 
        it->first->getShapeID(),
        HL_CLIENT_THREAD,
        HapticDevice::contactCallback );

      hlRemoveEventCallback( HL_EVENT_UNTOUCH, 
        it->first->getShapeID(),
        HL_CLIENT_THREAD,
        HapticDevice::separationCallback );

      hlRemoveEventCallback( HL_EVENT_MOTION, 
        it->first->getShapeID(),
        HL_CLIENT_THREAD,
        HapticDevice::motionCallback );

      it = m_contact_events.erase(it);
      //break;
    }
  }
}

bool HapticDevice::unRegisterContactEventHandler(Shape *shape)
{

  // If the device is shutting down, then ignore this unregister operation, otherwise we will 
  // mess up the iterators for the map holding the contact handlers
  if (m_shutting_down)
    return true;

  ShapeComposite *sc = dynamic_cast<ShapeComposite *> (shape);
  if (sc) {
    ShapeComposite::ShapeIDMap::iterator it = sc->begin();
    for(; it != sc->end(); it++) {
      unRegisterContactEventHandler(it->second);
    }
  }

   // remove from callback map AND shape_id_map
  ContactEventHandlerMap::iterator it = m_contact_events.find(shape);
  if (it != m_contact_events.end()) {

    hlRemoveEventCallback( HL_EVENT_TOUCH, 
      it->first->getShapeID(),
      HL_CLIENT_THREAD,
      HapticDevice::contactCallback );

    hlRemoveEventCallback( HL_EVENT_UNTOUCH, 
      it->first->getShapeID(),
      HL_CLIENT_THREAD,
      HapticDevice::separationCallback );

    hlRemoveEventCallback( HL_EVENT_MOTION, 
      it->first->getShapeID(),
      HL_CLIENT_THREAD,
      HapticDevice::motionCallback );

    m_contact_events.erase(it);
    return true;
  }
  return false;
}


Shape *HapticDevice::findShapeID(HLuint shape_id)
{
  ContactEventHandlerMap::const_iterator it = m_contact_events.begin();

  for(; it != m_contact_events.end(); it++) {
    if (*(it->first) == shape_id)
      return it->first;
  }
  return 0L;

  /*
  m_shape_id_map.find(shape_id);
  if (it == m_shape_id_map.end())
    return 0L;

  return (*it).second;
  */
}


void HapticDevice::registerContactEventHandler(Shape *shape, ContactEventHandler *cb, ContactState::ContactEvent event_type)
{
  assert(cb);
  assert(shape);

  m_contact_events[shape] = cb;

  if (event_type & ContactState::Contact) {

    ShapeComposite *sc = dynamic_cast<ShapeComposite *> (shape);
    if (sc) {
      ShapeComposite::ShapeIDMap::iterator it = sc->begin();
      for(; it != sc->end(); it++) {
        m_contact_events[it->second] = cb;

        hlAddEventCallback( HL_EVENT_TOUCH, 
          it->first,
          HL_CLIENT_THREAD,
          HapticDevice::contactCallback,
          this );
      }
    }
    else 
      hlAddEventCallback( HL_EVENT_TOUCH, 
        shape->getShapeID(),
        HL_CLIENT_THREAD,
        HapticDevice::contactCallback,
        this );
  }
  if (event_type & ContactState::Separation)
  {

    ShapeComposite *sc = dynamic_cast<ShapeComposite *> (shape);
    if (sc) {
      ShapeComposite::ShapeIDMap::iterator it = sc->begin();
      for(; it != sc->end(); it++) {
        m_contact_events[it->second] = cb;

        hlAddEventCallback( HL_EVENT_UNTOUCH, 
          it->first,
          HL_CLIENT_THREAD,
          HapticDevice::separationCallback,
          this );
      }
    }
    else {
    
      hlAddEventCallback( HL_EVENT_UNTOUCH, 
        shape->getShapeID(),
        HL_CLIENT_THREAD,
        HapticDevice::separationCallback,
        this  );    
    }
  }
  if (event_type & ContactState::Motion)
  {
    ShapeComposite *sc = dynamic_cast<ShapeComposite *> (shape);
    if (sc) {
      ShapeComposite::ShapeIDMap::iterator it = sc->begin();
      for(; it != sc->end(); it++) {
        m_contact_events[it->second] = cb;

        hlAddEventCallback( HL_EVENT_MOTION, 
          it->first,
          HL_CLIENT_THREAD,
          HapticDevice::motionCallback,
          this );
      }
    }
    else {
      hlAddEventCallback( HL_EVENT_MOTION, 
        shape->getShapeID(),
        HL_CLIENT_THREAD,
        HapticDevice::motionCallback,
        this );    
    }
  }
}

void HapticDevice::registerEventHandler(EventHandler *bev)
{
  m_event_handlers[bev] = bev;
}

/// Remove a callback object from the list of registrated button handlers
void HapticDevice::unRegisterEventHandler(EventHandler *bev)
{
  m_event_handlers[bev] = 0L;
}


#if 0
bool HapticDevice::calculateBartlettHanningWindowCoefficients(unsigned n, std::vector<float>& coefficients)
{

  
  w[1+k] = 0.62 - 0.48 | ((k/(n-1)) - 0.5)| + 0.38*cos(2PI(k/(n-1)-0.5))


}
#endif

bool HapticDevice::recalculateFilter()
{
  bool f = calculateFilterCoefficients(m_filter_coefficients, m_filter_window_size, m_cutoff, m_filter_type);
  if (f) {
    std::ofstream of;
    of.open("d:\\temp\asd.txt", std::ios_base::trunc);


    for(unsigned int i=0; i < m_filter_coefficients.size(); i++) 
      of << m_filter_coefficients[i] << std::endl;

  }

  return f;

}


/* Get the filter coefficients.  The user can choose between five different
   standard windowed-FIR filter models.
 */


bool HapticDevice::calculateFilterCoefficients( FilterCoefficients& coefficients, unsigned int M, double cutoff, enum FilterType filter_type )
{
  if (M <= 0 || M > 100) {
    osg::notify(osg::WARN) << "HapticDevice::calculateFilterCoefficients(): Invalid filter size specified" << std::endl;
    return false;
  }

    unsigned int i;
    double temp;
    double temp2;

    coefficients.resize(M);

    cutoff *= osg::PI;
    
    switch( filter_type )
    {
        case BARTLETT:
          //std::cerr << "Filter is Bartlett-windowed FIR lowpass " << M << " coefficients." << std::endl;
            for(i=0; i<M; i++)
            {
                temp = 2.0 * ((double)i)/((double)(M-1));        
                temp2 = ((double)i) - ((double)(M-1))/2.0;
                if( temp2 != 0.0 )
                {
                    if( ((double)i) <= ((double)(M-1))/2.0 )
                    {
                        coefficients[i] = temp * (sin(cutoff*temp2) / osg::PI / temp2);
                    }
                    else
                    {
                        coefficients[i] = (2.0 - temp) * 
                                (sin(cutoff*temp2) / osg::PI / temp2);
                    }
                }
                else
                {
                    coefficients[i] = cutoff / osg::PI;
                }
            }
            break;

        case HANNING:
            //std::cerr << "Filter is Hanning-windowed FIR lowpass " << M << " coefficients." << std::endl;
            for(i=0; i<M; i++)
            {
                temp = ((double)i)/((double)(M-1));
                temp2 = ((double)i) - ((double)(M-1))/2.0;
                if( temp2 != 0.0 )
                {
                    coefficients[i] = (0.5 - 0.5*cos(2.0*osg::PI*temp)) *
                            (sin(cutoff*temp2) / osg::PI / temp2);
                }
                else
                {
                    coefficients[i] = cutoff / osg::PI;
                }
            }
            break;
            
        case HAMMING:
            //std::cerr << "Filter is Hamming-windowed FIR lowpass " << M << " coefficients." << std::endl;
            for(i=0; i<M; i++)
            {
                temp = ((double)i)/((double)(M-1));
                temp2 = ((double)i) - ((double)(M-1))/2.0;
                if( temp2 != 0.0 )
                {
                    coefficients[i] = (0.54 - 0.46*cos(2.0*osg::PI*temp)) *
                            (sin(cutoff*temp2) / osg::PI / temp2);
                }
                else
                {
                    coefficients[i] = cutoff / osg::PI;
                }
            }
            break;
            
        case RECTANGULAR:
            //std::cerr << "Filter is Rectangular-windowed FIR lowpass " << M << " coefficients." << std::endl;
            for(i=0; i<M; i++)
            {
                temp2 = ((double)i) - ((double)(M-1))/2.0;
                if( temp2 != 0.0 )
                {
                    coefficients[i] = sin(cutoff*temp2) / osg::PI / temp2;
                }
                else
                {
                    coefficients[i] = cutoff / osg::PI;
                }
            }
            break;
            
                                
        default:
        case BLACKMAN:
            //std::cerr << "Filter is Blackman-windowed FIR lowpass " << M << " coefficients." << std::endl;
            for(i=0; i<M; i++)
            {
                temp = ((double)i)/((double)(M-1));
                temp2 = ((double)i) - ((double)(M-1))/2.0;
                if( temp2 != 0.0 )
                {
                    coefficients[i] = (0.42 - 0.5 * cos(2.0*osg::PI*temp) + 
                                0.08 * cos(4.0*osg::PI*temp)) *
                                (sin(cutoff*temp2) / osg::PI / temp2);
                }
                else
                {
                    coefficients[i] = cutoff / osg::PI;
                }
            }
            break;
    }
    return true;
}


/* Filter the data with a previously computed FIR filter.
 */
template<class T>
T HapticDevice::filter( std::vector<T>& in_data, 
                                FilterCoefficients& coefficients )
{
    unsigned int M = coefficients.size();

    if (in_data.size() < M) {
      warning("filter") << "Unable to calculated filtered value due to few sample points" << std::endl;
    }

    T sum;
    for(unsigned int i=0; i<M; i++)
    {
      sum += in_data[i] * coefficients[i];
    }
    return sum;
}

void HapticDevice::setProxyPosition(const osg::Vec3d& pos)
{
	osg::Matrix m;
  getWorldToWorkSpaceMatrix(m);
  //m.invert(m);
  osg::Vec3d ws_pos = m.preMult(pos);
//  hlDisable(HL_PROXY_RESOLUTION);
  hlProxydv(HL_PROXY_POSITION, ws_pos.ptr());
  hlProxyf(HL_STIFFNESS, m_proxy_stiffness);
  hlProxyf(HL_DAMPING, m_proxy_damping);
}

#if 0
HDCallbackCode HDCALLBACK HapticDevice::forceEffectCB( void *data ) {
  HapticDevice *device = static_cast< HapticDevice * >( data );

  // get current values from HD API 
  HLdouble m[16];
  hdGetDoublev( HD_CURRENT_TRANSFORM, m );
  osg::Matrix matrix(
    m[0], m[1], m[2], m[3],
    m[0], m[1], m[2], m[3],
    m[0], m[1], m[2], m[3],
    m[0], m[1], m[2], m[3]);

  hdGetDoublev( HD_CURRENT_VELOCITY, m );
  osg::Vec3 velocity( m[0],  m[1], m[2] );


  // add the resulting force and torque to the rendered force.
  osg::Vec3d force;
  hdGetDoublev( HD_CURRENT_FORCE,force.ptr() );

  HapticDevice::RenderForce rforce;

  // Transform the internal Hapticforce to a force in world coordinates
  force = device->m_touch_to_world_matrix.preMult(force);

  double current_time = device->getTimeStamp();
  device->m_log_stream << current_time;

  {  
    osg::Vec3 calculated_force;
    //      OpenThreads::ScopedLock<OpenThreads::Mutex> scope(device->m_mutex);
    if (device->getInterpolationMode() == NO_INTERPOLATION)
      calculated_force = device->m_current_render_force.getForce(); 

    else if (device->getInterpolationMode() == LINEAR_INTERPOLATION)
      calculated_force = device->m_current_render_force.evaluate(device->m_previous_render_force, current_time, false);

    else if (device->getInterpolationMode() == CUBICAL_INTERPOLATION)
      calculated_force = device->m_current_render_force.evaluate(device->m_previous_render_force, current_time, true);

    else if (device->getInterpolationMode() == FILTER_INTERPOLATION)
      calculated_force = device->filter(device->m_rendering_forces, device->m_filter_coefficients); //m_current_render_force.evaluate(device->m_previous_render_force, current_time, true);

    force += calculated_force;
    force = device->m_touch_to_world_matrix.postMult(force);
    device->m_log_stream << ";" << force[0] << ";" << force[1] << ";" << force[2] << ";" << force.length();

  }
  device->m_log_stream << std::endl;


  //device->getRenderForce(rforce);

  hdSetDoublev( HD_CURRENT_FORCE, force.ptr() );

  /*    HDdouble torque[3];
  hdGetDoublev( HD_CURRENT_TORQUE, torque );
  torque[0] += output.torque.x; 
  torque[1] += output.torque.y; 
  torque[2] += output.torque.z; 
  hdSetDoublev( HD_CURRENT_TORQUE, torque );
  */

  return HD_CALLBACK_CONTINUE;
}
#endif
