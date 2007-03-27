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

/*!
  Author: Anders Backman, VRlab Umeå University 060509

  This is a demo app that demonstrates how to draw objects both into the 
  haptic rendering pipeline as well as the visual.

  Dexterity is a classical test to measure precision and performance of movements
  in 3D. Move the ring without touching the wire.

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


#include <osgProducer/OsgSceneHandler>
#include <osgProducer/Viewer>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

#include <osg/Drawable>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>

#include <osg/Notify>


#include "dexterity.h"

using namespace sensors;





int main( int argc, char **argv )
{

  // use an ArgumentParser object to manage the program arguments.
  osg::ArgumentParser arguments(&argc,argv);

  // set up the usage document, in case we need to print out how to use this program.
  arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
  arguments.getApplicationUsage()->setDescription(arguments.getApplicationName()+" is the standard OpenSceneGraph example which loads and visualises 3d models.");
  arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");
  arguments.getApplicationUsage()->addCommandLineOption("--image <filename>","Load an image and render it on a quad");
  arguments.getApplicationUsage()->addCommandLineOption("--dem <filename>","Load an image/DEM and render it on a HeightField");
  arguments.getApplicationUsage()->addCommandLineOption("-h or --help","Display command line paramters");
  arguments.getApplicationUsage()->addCommandLineOption("--help-env","Display environmental variables available");
  arguments.getApplicationUsage()->addCommandLineOption("--help-keys","Display keyboard & mouse bindings available");
  arguments.getApplicationUsage()->addCommandLineOption("--help-all","Display all command line, env vars and keyboard & mouse bindigs.");


  // construct the viewer.
  osgProducer::Viewer viewer(arguments);
  viewer.getCullSettings();
  // set up the value with sensible default event handlers.
  viewer.setUpViewer(osgProducer::Viewer::STANDARD_SETTINGS);

  // get details on keyboard and mouse bindings used by the viewer.
  viewer.getUsage(*arguments.getApplicationUsage());

  // if user request help write it out to cout.
  bool helpAll = arguments.read("--help-all");
  unsigned int helpType = ((helpAll || arguments.read("-h") || arguments.read("--help"))? osg::ApplicationUsage::COMMAND_LINE_OPTION : 0 ) |
    ((helpAll ||  arguments.read("--help-env"))? osg::ApplicationUsage::ENVIRONMENTAL_VARIABLE : 0 ) |
    ((helpAll ||  arguments.read("--help-keys"))? osg::ApplicationUsage::KEYBOARD_MOUSE_BINDING : 0 );
  if (helpType)
  {
    arguments.getApplicationUsage()->write(std::cout, helpType);
    return 1;
  }

  // report any errors if they have occured when parsing the program aguments.
  if (arguments.errors())
  {
    arguments.writeErrorMessages(std::cout);
    return 1;
  }

  // read the scene from the list of file specified commandline args.
  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);

  // if no model has been successfully loaded report failure.
  if (!loadedModel.valid()) 
  {
    std::cout << arguments.getApplicationName() <<": No data loaded" << std::endl;
    return 1;
  }

  // any option left unread are converted into errors to write out later.
  arguments.reportRemainingOptionsAsUnrecognized();

  // report any errors if they have occured when parsing the program aguments.
  if (arguments.errors())
  {
    arguments.writeErrorMessages(std::cout);
  }

  // Overall scene root
  osg::ref_ptr<osg::Group> root = new osg::Group;


  // Root of the visual scene
  osg::ref_ptr<osg::Group> visual_root = new osg::Group;
  
  root->addChild(visual_root.get());


  // pass the loaded scene graph to the viewer.
  viewer.setSceneData(root.get());

  // create the windows and run the threads.
  viewer.realize();

  try {
    // Create a haptic device
    osg::ref_ptr<osgHaptics::HapticDevice> haptic_device = new osgHaptics::HapticDevice();
		haptic_device->createContext();			
		haptic_device->makeCurrent(); // Make this device the current one
    haptic_device->setEnableForceOutput(true); // Render output forces


    // Root of the haptic scene
    osgProducer::OsgSceneHandler* sceneHandler = viewer.getSceneHandlerList().front().get();
    osgUtil::SceneView *sceneView = sceneHandler->getSceneView();

    osg::ref_ptr<osgHaptics::HapticRootNode> haptic_root = new osgHaptics::HapticRootNode(sceneView);
    root->addChild(haptic_root.get());


    // Create a haptic material
    osg::ref_ptr<osgHaptics::Material> material = new osgHaptics::Material();

    //Set material attributes
    material->setStiffness(0.8);
    material->setDamping(0.2);
    material->setStaticFriction(0.7);
    material->setDynamicFriction(0.3);

    // Create a visitor that will prepare the Drawables in the subgraph so they can be rendered haptically
    // It merely attaches a osgHaptics::Shape ontop of each Drawable.
    // 
    // Prepare the node so that it can be rendered haptically
    // If several nodes are to be prepared, make sure to call the HapticRenderPrepareVisitor::reset() method
    // inbetween
    osgHaptics::HapticRenderPrepareVisitor vis(haptic_device.get());
    loadedModel->accept(vis);

    // Return the compound shape (can be used for contact tests, disabling haptic rendering for this subgraph etc.
    osgHaptics::Shape *shape = vis.getShape();
    osg::StateSet *ss = loadedModel->getOrCreateStateSet();

    // Store the shape.
    ss->setAttributeAndModes(shape);

    osg::ref_ptr<osgHaptics::TouchModel> touch = new osgHaptics::TouchModel();
    float snap_force_newtons = 10;
    touch->setMode(osgHaptics::TouchModel::CONTACT);
    touch->setSnapDistance(osgHaptics::TouchModel::calcForceToSnapDistance(snap_force_newtons));


    // specify a material attribute for this StateAttribute
    ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON);

    // specify a touch model for this StateAttribute
    ss->setAttributeAndModes(touch.get(), osg::StateAttribute::ON);


    // add it to the visual node to be rendered visually
    visual_root->addChild(loadedModel.get());

    // Add it to the haptic root
    haptic_root->addChild(loadedModel.get());

    // create dexteritySpline()
    DexterityLineDrawable *line = createDexteritySpline(visual_root.get(), haptic_root.get(), haptic_device.get());

    // Create a proxy node for rendering the deviceposition
    osg::ref_ptr<osg::Node> proxy_sphere = osgDB::readNodeFile("dexterity_pin.ac"); // Load a pen geometry
    osg::ref_ptr<osg::MatrixTransform> proxy_transform = new osg::MatrixTransform;    
    proxy_transform->addChild(proxy_sphere.get());

    visual_root->addChild(proxy_transform.get());



    osg::BoundingBox bbox=line->getBound();

    /*
    Set the workspace of the Haptic working area to enclose the bounding box of the scene.
    Also, set the WorkSpace mode to use the Bounding box
    Notice that the haptic device will not follow the camera as it does by default. (VIEW_MODE)
    */
    haptic_device->setWorkspace(bbox._min, bbox._max);
    haptic_device->setWorkspaceMode(osgHaptics::HapticDevice::BBOX_MODE);


    // Create an OsgSensor that will read transformations from the HapticDevice and feedback to
    // a SensorCallback, that will update a certain node.
    // So we have: device->osgSensor->SensorCallback->MatrixTransformation
    //
    osg::ref_ptr<osgSensor::OsgSensor> sensor = new osgSensor::OsgSensor(haptic_device.get());
    osg::ref_ptr<osgSensor::OsgSensorCallback> sensor_callback = new osgSensor::OsgSensorCallback(sensor.get());

    proxy_transform->setUpdateCallback(sensor_callback.get());

    // Add a custom drawable that draws the rendered force of the device
    osg::Geode *geode = new osg::Geode;
    VectorDrawable *force_drawable = new VectorDrawable();
    geode->addDrawable(force_drawable);
    visual_root->addChild(geode);

    // Add a custom drawable that will draw a line between the closest point on the wire and the proxy
    VectorDrawable *vector = new VectorDrawable();
    vector->setColor(0.2,0.7,0.7);
    {
      osg::Geode *geode = new osg::Geode;
      geode->addDrawable(vector);
      visual_root->addChild(geode);
    }


    /*
    Add pre and post draw callbacks to the camera so that we start and stop a haptic frame
    at a time when we have a valid OpenGL context.
    */
    osgHaptics::prepareHapticCamera(viewer.getCamera(0), haptic_device.get(), root.get());

    while( !viewer.done() )
    {
      // wait for all cull and draw threads to complete.
      viewer.sync();        

      // Update all registrated sensors (HapticDevice) is one.
      // We could just call haptic_device->update() in a nodecallback somewhere if we want
      // or attach a UpdateDeviceCallback to any node in the scenegraph...
      // But be sure to update the device BEFORE its transformation is used.
      // otherwise the transformation will lag one frame
      //
      g_SensorMgr->update();
      osg::Vec3 pos = haptic_device->getProxyPosition();
      osg::Vec3 closest;
      float distance = line->distance(pos, closest);
      vector->set(pos, closest);

      // update the scene by traversing it with the the update visitor which will
      // call all node update callbacks and animations.
      viewer.update();

      // Render the force vector
      {
        osg::Vec3 start, end;

        start = haptic_device->getProxyPosition();
        osg::Vec3 force = haptic_device->getForce();
        float len = force.length();

        force.normalize();
        osg::Vec3 dir = force;

        // Update the force drawable with the current force of the device
        force_drawable->set(start, start+dir*len);
      }

      // fire off the cull and draw traversals of the scene.
      viewer.frame();        
    }

    // wait for all cull and draw threads to complete before exit.
    viewer.sync();

    // Shutdown all registrated sensors
    osgSensor::SensorMgr::instance()->shutdown();

  } catch (std::exception& e) {
    std::cerr << "Caught exception: " << e.what() << std::endl;
  }

  return 0;
}
