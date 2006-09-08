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
  Author: Anders Backman, VRlab, Umeå University 2006-05-15

  
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
#include <osgGA/GUIEventHandler>
#include <osgGA/GUIEventAdapter>

#include <osg/Notify>

using namespace sensors;


class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:

  KeyboardEventHandler(osgHaptics::HapticSpringNode *spring_node) : m_spring_node(spring_node) {}

  virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&);
    

  virtual void accept(osgGA::GUIEventHandlerVisitor& v)
  {
    v.visit(*this);
  }

private:
  osg::ref_ptr<osgHaptics::HapticSpringNode> m_spring_node;

};

bool KeyboardEventHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
{
  switch(ea.getEventType())
  {
  case(osgGA::GUIEventAdapter::KEYDOWN):
    {
      if (ea.getKey() == osgGA::GUIEventAdapter::KEY_F1) {
        m_spring_node->setEnable(!m_spring_node->getEnable());
        std::string str = m_spring_node->getEnable() ? "Enabled" : "Disabled";
        osg::notify(osg::WARN) << str << " rendering of spring" << std::endl;
        return true;
      }
   
      if (ea.getKey() == osgGA::GUIEventAdapter::KEY_F2) {
        osgHaptics::SpringForceOperator *spring = m_spring_node->getForceOperator();
        if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_SHIFT)
          spring->setStiffness(spring->getStiffness()*0.9);
        else
          spring->setStiffness(spring->getStiffness()*1.1);

        osg::notify(osg::WARN) << "Stiffness : " << spring->getStiffness() << std::endl;
        return true;
      }

      return false;
    }
  default:
    return false;
  }
  return false;
}


/* Simple class for drawing the forcevector */
class VectorDrawable : public osg::Drawable{
public:
  VectorDrawable() : Drawable() {
    setUseDisplayList(false);
    setColor(1.0f,0.0f,0.0f);
    set(osg::Vec3(0,0,0), osg::Vec3(1,1,1));
  }

  VectorDrawable(const VectorDrawable& drawable, 
    const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY) : 
  osg::Drawable(drawable, copyop) {};

  META_Object(osg,VectorDrawable)


    void set(const osg::Vec3& start, const osg::Vec3& end) {
      m_start = start; m_end = end;
      dirtyBound();
    }

    void setColor(float r, float g, float b ) {  m_color.set(r,g,b); }

    void drawImplementation(osg::State& state) const
    {
      glPushAttrib(GL_ALL_ATTRIB_BITS);
      glDisable(GL_LIGHTING);
      glColor3fv( m_color.ptr() );

      glBegin(GL_LINES);
      glVertex3fv(m_start.ptr());
      glVertex3fv(m_end.ptr());  
      glEnd();

      glPopAttrib();
    }

    osg::BoundingBox computeBound() const 
    {
      osg::BoundingBox bbox;
      bbox.expandBy(m_start);
      bbox.expandBy(m_end);
      return bbox;
    }
private:
  osg::Vec3 m_color;
  osg::Vec3 m_start, m_end;
};

osg::Node *createHud() 
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
  text->setText("F1 - Toggle spring, F2/Shift+F2 - Increase/decrease stiffness");


  osg::CameraNode* camera = new osg::CameraNode;

  // set the projection matrix
  camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1,0,1));

  // set the view matrix    
  camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  camera->setViewMatrix(osg::Matrix::identity());

  // only clear the depth buffer
  camera->setClearMask(GL_DEPTH_BUFFER_BIT);

  // draw subgraph after main camera view.
  camera->setRenderOrder(osg::CameraNode::POST_RENDER);

  camera->addChild(geode);
  return camera;
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

  root->addChild(createHud());
  // pass the loaded scene graph to the viewer.
  viewer.setSceneData(root.get());

  // create the windows and run the threads.
  viewer.realize();

  try {
    // Create a haptic device
    osg::ref_ptr<osgHaptics::HapticDevice> haptic_device = new osgHaptics::HapticDevice();
    haptic_device->init(); // Initialize it
    haptic_device->makeCurrent(); // Make this device the current one
    haptic_device->setEnableForceOutput(true); // Render output forces


    // Root of the haptic scene
    osgProducer::OsgSceneHandler* sceneHandler = viewer.getSceneHandlerList().front().get();
    osgUtil::SceneView *sceneView = sceneHandler->getSceneView();

    osg::ref_ptr<osgHaptics::HapticRootNode> haptic_root = new osgHaptics::HapticRootNode(sceneView);
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
    osgHaptics::HapticRenderPrepareVisitor vis(haptic_device.get());
    loadedModel->accept(vis);

    // Return the compound shape (can be used for contact tests, disabling haptic rendering for this subgraph etc.
    osgHaptics::Shape *shape = vis.getShape();
    osg::StateSet *ss = loadedModel->getOrCreateStateSet();

    // Store the shape.
    ss->setAttributeAndModes(shape);

    // specify a material attribute for this StateAttribute
    ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON);


    /*
      Create a HapticSpringNode and make that follow the animated sphere
    */
    {
      osgSensor::FindVisitor fv;
      osg::Node * node = fv.find("animated_sphere", visual_root.get());
      if (!node) {
        osg::notify(osg::FATAL) << "\n*** Unable to find a node named animated_sphere in the loaded scene" << std::endl;
        osg::notify(osg::FATAL) << "*** make sure you read the documentation for the example application and load the right model" << std::endl;
        throw std::runtime_error("Unable to find a node named animated_sphere in the loaded scene");
      }
      osg::Group *group = dynamic_cast<osg::Group *>(node);
      if (!group) {
        osg::notify(osg::FATAL) << "\n***Found a node named animated_sphere but its not derived from Group class" << std::endl;
        osg::notify(osg::FATAL) << "*** make sure you read the documentation for the example application and load the right model" << std::endl;
        throw std::runtime_error("Found a node named animated_sphere but its not derived from Group class");
      }

      osgHaptics::HapticSpringNode *spring_node =new osgHaptics::HapticSpringNode(haptic_device.get());
      group->addChild(spring_node);
      spring_node->setEnable(false);

      // Create a keyboard handler that will react to keyboard events
      KeyboardEventHandler *key_handler = new KeyboardEventHandler(spring_node);
      viewer.getEventHandlerList().push_front(key_handler);
    }


    // Create a proxy sphere for rendering
    osg::ref_ptr<osg::Node> proxy_sphere = osgDB::readNodeFile("pen.ac"); // Load a pen geometry
    osg::ref_ptr<osg::MatrixTransform> proxy_transform = new osg::MatrixTransform;    
    proxy_transform->addChild(proxy_sphere.get());

    visual_root->addChild(proxy_transform.get());

    /*
    Add pre and post draw callbacks to the camera so that we start and stop a haptic frame
    at a time when we have a valid OpenGL context.
    */
    osgHaptics::prepareHapticCamera(viewer.getCamera(0), haptic_device.get(), root.get());

    /*
    Get the bounding box of the loaded scene
    */
    osg::BoundingSphere bs = visual_root->getBound();
    float radius = bs.radius();

    /*
    There are many ways to specify a working volume for the haptics.
    One is to use the ViewFrustum.
    This requires use to set the VF to a limited volume so we dont get extensive scalings in any axis.
    Usually the far-field is set to something like 1000, which is not appropriate.

    Below is a way to restrain the near and far field.
    setWorkspaceMode(VIEW_MODE) will effectively use the viewfrustum as a haptic workspace.
    This will also make the haptic device follow the camera
    */
    //haptic_device->setWorkspaceMode(osgHaptics::HapticDevice::VIEW_MODE);


    osg::BoundingBox bbox;
    bbox.expandBy(osg::Vec3(4,3,1.5)*(-0.5));
    bbox.expandBy(osg::Vec3(4,3,1.5)*(0.5));
  
     /*
    Another way is to set the workspace of the Haptic working area to enclose the bounding box of the scene.
    Also, set the WorkSpace mode to use the Bounding box
    Notice that the haptic device will not follow the camera as it does by default. (VIEW_MODE)
    */
    haptic_device->setWorkspace(bbox._min, bbox._max);
    haptic_device->setWorkspaceMode(osgHaptics::HapticDevice::BBOX_MODE);


    // create sensors that connects the device to a transformation-node 
    osg::ref_ptr<osgSensor::OsgSensor> sensor = new osgSensor::OsgSensor(haptic_device.get());
    osg::ref_ptr<osgSensor::OsgSensorCallback> sensor_callback = new osgSensor::OsgSensorCallback(sensor.get());

    proxy_transform->setUpdateCallback(sensor_callback.get());

    bool set_pos = false;

    


    // Add a custom drawable that draws the rendered force of the device
    osg::Geode *geode = new osg::Geode;
    VectorDrawable *force_drawable = new VectorDrawable();
    geode->addDrawable(force_drawable);
    visual_root->addChild(geode);


    while( !viewer.done() )
    {
      // wait for all cull and draw threads to complete.
      viewer.sync();        

      // Update all registrated sensors (HapticDevice) is one.
      g_SensorMgr->update();

      // update the scene by traversing it with the the update visitor which will
      // call all node update callbacks and animations.
      viewer.update();

      osg::Vec3 start, end;

      start = haptic_device->getProxyPosition();
      osg::Vec3 force = haptic_device->getForce();
      float len = force.length();

      force.normalize();
      osg::Vec3 dir = force;

      // Update the force drawable with the current force of the device
      force_drawable->set(start, start+dir*len);

      // fire off the cull and draw traversals of the scene.
      viewer.frame();        
    }

    // wait for all cull and draw threads to complete before exit.
    viewer.sync();


  } catch (std::exception& e) {
    osg::notify(osg::FATAL) << "Caught exception: " << e.what() << std::endl;
  }

  // Shutdown all registrated sensors
  osgSensor::SensorMgr::instance()->shutdown();

  return 0;
}
