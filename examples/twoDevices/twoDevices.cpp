/*  This progamm is a simple project using two Phantom Omni device. 
*		The scene contains two ball for two Phantom Omni device to touch.
*
*
* Sophia Soo, CUHK/Dep. Surgery/Jan2007
*/

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

#include <osgSensor/OsgSensorCallback.h>
#include <osgSensor/SensorMgr.h>

#include <osgHaptics/HapticDevice.h>
#include <osgHaptics/osgHaptics.h>
#include <osgHaptics/Shape.h>
#include <osgHaptics/VibrationForceOperator.h>
#include <osgHaptics/ForceEffect.h>
#include <osgHaptics/HapticRootNode.h>
#include <osgHaptics/HapticRenderPrepareVisitor.h>
#include <osgHaptics/SpringForceOperator.h>
#include <osgHaptics/TouchModel.h>
#include <osgHaptics/Shape.h>
#include <osgHaptics/Material.h>
#include <osgHaptics/BBoxVisitor.h>

#include <osgViewer/Viewer>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/KeySwitchMatrixManipulator>


#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

#include <osg/PolygonOffset>
#include <osg/Drawable>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>

#include <osg/Notify>

#include  <osgHaptics/HashedGrid.h>
#include  <osgHaptics/HashedGridDrawable.h>
#include  <osgHaptics/TriangleExtractor.h>

#include <osg/ref_ptr>
#include <time.h>
#include <iostream>
#include <osg/io_utils>

// Define this to use two phantoms
#define ARMSTWOTEST

using namespace sensors;


// Create two cameras, one for each haptic context
void setupCameras( osgViewer::Viewer& viewer )
{
	osg::Camera* cam1 = viewer.getCamera();
	osg::Camera* cam2 = new osg::Camera( *cam1 );
	viewer.addSlave(cam2);

   /*
   Producer::CameraConfig* myCameraConfig = new Producer::CameraConfig();
   Producer::Camera* camera1 = new Producer::Camera();
   Producer::Camera* camera2 = new Producer::Camera();

   camera1->setShareLens(true);
   camera1->getLens()->setAutoAspect(true);

   camera2->setShareLens(true);
   camera2->getLens()->setAutoAspect(true);

   myCameraConfig->addCamera("camera1",camera1);
   myCameraConfig->addCamera("camera2",camera2);

   Producer::RenderSurface* rsOne = camera1->getRenderSurface();
   camera2->setRenderSurface( rsOne );
   return myCameraConfig;
   */
}

osg::MatrixTransform* createSphereX(float radius, osg::Vec4 color) {

	osg::Sphere* unitSphere = new osg::Sphere( osg::Vec3(0,0,0), radius);
	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
	unitSphereDrawable->setColor( color );

	osg::MatrixTransform* unitSphereXForm = new osg::MatrixTransform();
	osg::Geode* unitSphereGeode = new osg::Geode();
	unitSphereXForm->addChild(unitSphereGeode);
	unitSphereGeode->addDrawable(unitSphereDrawable);

	return unitSphereXForm;
}


osg::PositionAttitudeTransform* createSceneBall(float radius, osg::Vec4 color) {

	osg::Sphere* unitSphere = new osg::Sphere( osg::Vec3(0,0,0), radius);
	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
	unitSphereDrawable->setColor( color );
	osg::PositionAttitudeTransform* unitSphereXForm = new osg::PositionAttitudeTransform();
	osg::Geode* unitSphereGeode = new osg::Geode();
	unitSphereXForm->addChild(unitSphereGeode);
	unitSphereGeode->addDrawable(unitSphereDrawable);

	return unitSphereXForm;
}



int main( int argc, char **argv )
{
	//-----------------------------------------------------------------------------------------
	// set up viewer
	//-----------------------------------------------------------------------------------------

	osgViewer::Viewer viewer;
	// Set threading model to SingleThreaded, otherwise we will get a hang in OpenHaptics.
	viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded); // CullDrawThreadPerContext also works

	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
	keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
	viewer.setCameraManipulator( keyswitchManipulator.get() );


	// add the window size toggle handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	viewer.addEventHandler(new osgViewer::StatsHandler);




#ifdef ARMSTWOTEST
    //-- create 2 cameras for the viewer
	setupCameras( viewer );
#endif

    //viewer.getCullSettings();
	// value with sensible default event handlers
    //viewer.setUpViewer(osgViewer::Viewer::STANDARD_SETTINGS); 

	//-----------------------------------------------------------------------------------------
	// create model
	//-----------------------------------------------------------------------------------------

	osg::ref_ptr<osg::Group> loadedModel = new osg::Group;

	//-- create first ball
	osg::PositionAttitudeTransform* firstBallTransform = createSceneBall(3.5, osg::Vec4(0.5, 0.2, 0.5, 0.5));
	firstBallTransform->setPosition(osg::Vec3(0, 0, 0));
	loadedModel->addChild(firstBallTransform);

	//-- create second ball
	osg::PositionAttitudeTransform* secondBallTransform = createSceneBall(2.5, osg::Vec4(0.5, 0.8, 0.5, 0.5));
	secondBallTransform->setPosition(osg::Vec3(10, 0, 0));
	loadedModel->addChild(secondBallTransform);


	//-----------------------------------------------------------------------------------------
	// create scene root for viewer
	//-----------------------------------------------------------------------------------------
	//-- create scene root
	osg::ref_ptr<osg::Group> root = new osg::Group;

	//-- Root of the visual scene
	osg::ref_ptr<osg::Group> visual_root = new osg::Group;
	root->addChild(visual_root.get());
  
	//-- pass the loaded scene graph to the viewer.
	viewer.setSceneData(root.get());

	//-- create the windows and run the threads.
	viewer.realize();

	try {

		//-----------------------------------------------------------------------------------------
		// Create Two haptic devices 
		// The order in which devices are created and initialized is important. 
		// Look at the code below
		//-----------------------------------------------------------------------------------------

#ifdef ARMSTWOTEST

		//-- Create the first, default device
		osg::ref_ptr<osgHaptics::HapticDevice> haptic_device = new osgHaptics::HapticDevice();

		// Create the second device
		// The id used for the device can differ between different devices, so make sure
		// it matches the one set in the controlpanel for the phantoms
		osg::ref_ptr<osgHaptics::HapticDevice> haptic_device2 = new osgHaptics::HapticDevice("Phantom2");

		//-- create context : must initialize all device before create context)
		haptic_device->createContext();			
		haptic_device->makeCurrent();				// Make this device the current one
		haptic_device->setEnableForceOutput(true);	// Render output forces 

		haptic_device2->createContext();	
		haptic_device2->makeCurrent();				// Make this device the current one
		haptic_device2->setEnableForceOutput(true);	// Render output forces 
#else

		//-- Initialize 
		osg::ref_ptr<osgHaptics::HapticDevice> haptic_device = new osgHaptics::HapticDevice();
		haptic_device->createContext();			
		haptic_device->makeCurrent();				// Make this device the current one
		haptic_device->setEnableForceOutput(true);	// Render output forces 

#endif
 		//-----------------------------------------------------------------------------------------
		// Create Root of the haptic scene
		//-----------------------------------------------------------------------------------------
		osg::Camera *camera = viewer.getCamera();
		osg::ref_ptr<osgHaptics::HapticRootNode> haptic_root = new osgHaptics::HapticRootNode(camera);
		//osg::ref_ptr<osgHaptics::HapticRootNode> haptic_root = new osgHaptics::HapticRootNode(&viewer);
		root->addChild(haptic_root.get());

 		//-----------------------------------------------------------------------------------------
		// construct scene
		//-----------------------------------------------------------------------------------------
		//-- load the model containing the sourrounding objects into the visual and haptic environment
		osg::Node *visual_node = loadedModel.get();
		osg::Node *haptic_node = loadedModel.get();

 		//-----------------------------------------------------------------------------------------
		// load the model containing the sourrounding objects into the visual and haptic environment
		//-----------------------------------------------------------------------------------------
		//-- add it to the visual node to be rendered visually
		visual_root->addChild(visual_node);

		//-- add it to the haptic root
		haptic_root->addChild(haptic_node);

		//---------------------------------------------------------------------------------
		// Sphere represent the tip of the devices
		//---------------------------------------------------------------------------------
		osg::MatrixTransform* tipTransform_device1 = createSphereX(0.5, osg::Vec4(0.8, 0.1, 0.1, 0.1));
		visual_root->addChild(tipTransform_device1);

#ifdef ARMSTWOTEST
		osg::MatrixTransform* tipTransform_device2 = createSphereX(0.5, osg::Vec4(0.1, 0.1, 0.9, 0.1));
		visual_root->addChild(tipTransform_device2);
#endif

		//---------------------------------------------------------------------------------
		// Add pre and post draw callbacks to the camera so that we start and stop a haptic frame
		// at a time when we have a valid OpenGL context.
		//---------------------------------------------------------------------------------
	    //osgHaptics::prepareHapticCamera(&viewer, haptic_device.get(), 0, root.get());
		osgViewer::Viewer::Cameras cameras;
		viewer.getCameras( cameras );
		osgHaptics::prepareHapticCamera(cameras[0], haptic_device.get(), root.get());

#ifdef ARMSTWOTEST
		//osgHaptics::prepareHapticCamera(&viewer, haptic_device2.get(), 1, root.get());
		osgHaptics::prepareHapticCamera(cameras[1], haptic_device2.get(), root.get());
#endif

		//---------------------------------------------------------------------------------
		// set the workspace for devices
		//---------------------------------------------------------------------------------

		//-- Get the bounding box of the loaded scene
		osg::BoundingBox bbox;
		osgHaptics::BBoxVisitor bv;
		haptic_root->accept(bv);
		bbox = bv.getBoundingBox();

		//-- Another way is to set the workspace of the Haptic working area to enclose the bounding box of the scene.
		//-- Also, set the WorkSpace mode to use the Bounding box
		//-- Notice that the haptic device will not follow the camera as it does by default. (VIEW_MODE)
		haptic_device->setWorkspace(bbox._min, bbox._max);
		haptic_device->setWorkspaceMode(osgHaptics::HapticDevice::BBOX_MODE);

#ifdef ARMSTWOTEST
	    haptic_device2->setWorkspace(bbox._min, bbox._max);
		haptic_device2->setWorkspaceMode(osgHaptics::HapticDevice::BBOX_MODE);
#endif

		//-- A transformation using the TouchWorkSpace matrix will also scale all the force/position and 
		//-- orientation output from the haptic device.
		//-- It will also affect the size, orientation and position of the haptic workspace.
		//-- Practical if you want to rotate the workspace relative to the device.
		osg::Matrixd tm;
		float workspace_scale = 5.0;
		tm.makeScale(workspace_scale, workspace_scale, workspace_scale);
		haptic_device->setTouchWorkspaceMatrix(tm);

#ifdef ARMSTWOTEST
	    haptic_device2->setTouchWorkspaceMatrix(tm);
#endif

		//---------------------------------------------------------------------------------
		// create osgsensor to get the Phantom position and update the system
		//---------------------------------------------------------------------------------

		//--DEVICE 1
		//JP The sensor is my PhantomOmni haptic device [senses position and orientation]
		osg::ref_ptr<osgSensor::OsgSensor> sensor = new osgSensor::OsgSensor(haptic_device.get());
    
		// The sensor_callback allways gets/updates the Phantom position for visual rendering
		osg::ref_ptr<osgSensor::OsgSensorCallback> sensor_callback = new osgSensor::OsgSensorCallback(sensor.get());
	
		//@x: now the sensor and sensor_callback exists. This callback will update the 
		//grasper system position and orientation
		tipTransform_device1->setUpdateCallback(sensor_callback.get());

#ifdef ARMSTWOTEST
		//--DEVICE 2
		//JP The sensor is my PhantomOmni haptic device [senses position and orientation]
		osg::ref_ptr<osgSensor::OsgSensor> sensor2 = new osgSensor::OsgSensor(haptic_device2.get());
    
		// The sensor_callback allways gets/updates the Phantom position for visual rendering
		osg::ref_ptr<osgSensor::OsgSensorCallback> sensor_callback2 = new osgSensor::OsgSensorCallback(sensor2.get());
	
		//@x: now the sensor and sensor_callback exists. This callback will update the 
		//grasper system position and orientation
		tipTransform_device2->setUpdateCallback(sensor_callback2.get());
#endif
		//---------------------------------------------------------------------------------
		// create visitor to attach shape for drawables under subgraph
		//---------------------------------------------------------------------------------

		//--DEVICE 1
		osgHaptics::HapticRenderPrepareVisitor vis(haptic_device.get());
		haptic_node->accept(vis);

#ifdef ARMSTWOTEST

		//--DEVICE 2
		osgHaptics::HapticRenderPrepareVisitor vis2(haptic_device2.get());
		haptic_node->accept(vis2);

#endif
		while( !viewer.done() )
		{
			// wait for all cull and draw threads to complete.
//			viewer.sync();        

			// Update all registrated sensors (HapticDevice) is one.
			g_SensorMgr->update();

			// update the scene by traversing it with the the update visitor which will
			// call all node update callbacks and animations.
//			viewer.update();

			// fire off the cull and draw traversals of the scene.
			viewer.frame();

		}//while (!viewer.done())
	  
		// wait for all cull and draw threads to complete before exit.
//		viewer.sync();

		// Shutdown all registrated sensors
		osgSensor::SensorMgr::instance()->shutdown();

	
	} catch (std::exception& e) {
		osg::notify(osg::FATAL) << "Caught exception: " << e.what() << std::endl;
		osg::notify(osg::FATAL) << "Make sure you are using the right id for the phantoms." << e.what() << std::endl;
	} //try

	return 0;

};
