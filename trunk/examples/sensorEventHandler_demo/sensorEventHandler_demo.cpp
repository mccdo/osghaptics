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
  Author: Anders Backman, VRlab, Umeå University 2006-11-08
  Demonstrates the SensorEventHandler
  
*/


#include <osgSensor/OsgSensorCallback.h>
#include <osgSensor/SensorMgr.h>


#include <osgText/Text>
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
#include <osgHaptics/HapticSpringNode.h>
#include <osgSensor/Visitors.h>

#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/KeySwitchMatrixManipulator>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

#include <osg/Drawable>
#include <osg/PositionAttitudeTransform>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/PolygonMode>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>

#include <osg/Notify>

#include <osgSensor/SensorEventHandler.h>

using namespace osgSensor;


class MySensorEventHandler : public SensorEventHandler
{
public:
  MySensorEventHandler(osgText::Text *text) : m_text(text) {}

  virtual void operator()(EventType eventType, float time);

private:
  osg::ref_ptr<osgText::Text> m_text;
};

void
MySensorEventHandler::operator ()(EventType eventType, float time)
{
  // We just ignore the update EVENT!
  if (eventType == UPDATE)
    return;

  char *buttonStrings[] = { "BUTTON_NONE", "BUTTON_1", "BUTTON_2" };
  char *buttonState[] = { "STATE_NONE", "DOWN", "UP"};

  std::cerr << osgHaptics::HapticDevice::CALIBRATION_UPDATE_EVENT << std::endl;
  std::ostringstream str;
  str << "Time: " << time;
  if ( eventType == BUTTON ) {
    Button button = getButton();
    
    ButtonState state = getButtonState(button);
    str  << " Event: BUTTON, Button: " << buttonStrings[button] << " State: " << buttonState[state] << std::endl;
  }

  else if (eventType == osgHaptics::HapticDevice::CALIBRATION_UPDATE_EVENT) {
    str << "Event CALIBRATION_UPDATE_EVENT";
  }

  else if (eventType == osgHaptics::HapticDevice::CALIBRATION_INPUT_EVENT) {
    str << "Event CALIBRATION_INPUT_EVENT";
  }
  m_text->setText(str.str());
}

osgText::Text *createHud(osg::Group *root) 
{
  osg::Geode *geode = new osg::Geode;

  std::string timesFont("fonts/arial.ttf");

  // turn lighting off for the text and disable depth test to ensure its always ontop.
  osg::StateSet* stateset = geode->getOrCreateStateSet();
  stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

  osg::Vec3 position(0.1, 0.05,0);

  osgText::Text* text = new  osgText::Text;
  text->setCharacterSize(0.03);
  geode->addDrawable( text );

  text->setFont(timesFont);
  text->setPosition(position);
  text->setText("");

  osg::Camera* camera = new osg::Camera;

  // set the projection matrix
  camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1,0,1));

  // set the view matrix    
  camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  camera->setViewMatrix(osg::Matrix::identity());

  // only clear the depth buffer
  camera->setClearMask(GL_DEPTH_BUFFER_BIT);

  // draw subgraph after main camera view.
  camera->setRenderOrder(osg::Camera::POST_RENDER);

  camera->addChild(geode);
  root->addChild(camera);
  return text;
}

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
  arguments.getApplicationUsage()->addCommandLineOption("-h or --help","Display command line parameters");
  arguments.getApplicationUsage()->addCommandLineOption("--help-env","Display environmental variables available");
  arguments.getApplicationUsage()->addCommandLineOption("--help-keys","Display keyboard & mouse bindings available");
  arguments.getApplicationUsage()->addCommandLineOption("--help-all","Display all command line, env vars and keyboard & mouse bindigs.");
  arguments.getApplicationUsage()->addKeyboardMouseBinding("F1","Toggle haptic spring on off");
  arguments.getApplicationUsage()->addKeyboardMouseBinding("F2","Increase spring stiffness");
  arguments.getApplicationUsage()->addKeyboardMouseBinding("F2+Shift","Decrease spring stiffness");

  // construct the viewer.
  osgViewer::Viewer viewer(arguments);

	// Set threading model to SingleThreaded, otherwise we will get a hang in OpenHaptics.
	viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded); // CullDrawThreadPerContext also works

	osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
	keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
	viewer.setCameraManipulator( keyswitchManipulator.get() );


	// add the window size toggle handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	// add the stats handler
	viewer.addEventHandler(new osgViewer::StatsHandler);

	// add the help handler
	viewer.addEventHandler(new osgViewer::HelpHandler(arguments.getApplicationUsage()));


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

  // optimize the scene graph, remove redundant nodes and state etc.
  osgUtil::Optimizer optimizer;
  optimizer.optimize(loadedModel.get());

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
//    osgProducer::OsgSceneHandler* sceneHandler = viewer.getSceneHandlerList().front().get();
    osg::Camera *camera = viewer.getCamera();//sceneHandler->getSceneView();

    osg::ref_ptr<osgHaptics::HapticRootNode> haptic_root = new osgHaptics::HapticRootNode(camera);
    root->addChild(haptic_root.get());

    // add it to the visual node to be rendered visually
    visual_root->addChild(loadedModel.get());

    // Add it to the haptic root
    haptic_root->addChild(loadedModel.get());


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

    // specify a material attribute for this StateAttribute
    ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON);

    // Create a proxy sphere for rendering
    osg::ref_ptr<osg::Node> proxy_sphere = osgDB::readNodeFile("pen.ac"); // Load a pen geometry
    osg::ref_ptr<osg::MatrixTransform> proxy_transform = new osg::MatrixTransform;    
    proxy_transform->addChild(proxy_sphere.get());

    visual_root->addChild(proxy_transform.get());

    // create sensors that connects the device to a transformation-node 
    osg::ref_ptr<osgSensor::OsgSensor> sensor = new osgSensor::OsgSensor(haptic_device.get());
    osg::ref_ptr<osgSensor::OsgSensorCallback> sensor_callback = new osgSensor::OsgSensorCallback(sensor.get());

    proxy_transform->setUpdateCallback(sensor_callback.get());

    bool set_pos = false;


    /*
    Add pre and post draw callbacks to the camera so that we start and stop a haptic frame
    at a time when we have a valid OpenGL context.
    */
    osgHaptics::prepareHapticCamera(viewer.getCamera(), haptic_device.get(), root.get());


    osgText::Text *text = createHud(root.get());

    // Create and registrate our SensorEventHandler that will react to button presses
    // and other events that the Sensor (HapticDevice) will trigger.
    MySensorEventHandler *sensor_handler = new MySensorEventHandler(text);
    haptic_device->registerSensorEventHandler(sensor_handler);

    while( !viewer.done() )
    {
      // wait for all cull and draw threads to complete.
//      viewer.sync();        

      // Update all registrated sensors (HapticDevice) is one.
      osg::FrameStamp *frame_stamp = viewer.getFrameStamp();
      double time = frame_stamp->getReferenceTime();
      g_SensorMgr->update(time);

      // update the scene by traversing it with the the update visitor which will
      // call all node update callbacks and animations.
//      viewer.update();

      osg::Vec3 start, end;

      start = haptic_device->getProxyPosition();
      osg::Vec3 force = haptic_device->getForce();
      float len = force.length();

      force.normalize();
      osg::Vec3 dir = force;


      // fire off the cull and draw traversals of the scene.
      viewer.frame();        
    }

    // wait for all cull and draw threads to complete before exit.
//    viewer.sync();


  } catch (std::exception& e) {
    osg::notify(osg::FATAL) << "Caught exception: " << e.what() << std::endl;
  }

  // Shutdown all registrated sensors
  osgSensor::SensorMgr::instance()->shutdown();

  return 0;
}
