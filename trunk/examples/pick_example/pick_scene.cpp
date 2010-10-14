/*!
  Date: 060911
  Author: Anders Backman
  Purpose: Create a scene that allows a user to touch a haptic node (containing a shape) and move it around.
  Hopefully the code is self-explanitory
*/
#include "pick_scene.h"



ButtonEventHandler::ButtonEventHandler(ContactCallback *cb, osg::MatrixTransform *proxy_node) : m_cb(cb) 
{
  // Create a transformation sensor, its a sensor wich will read a transformation from a MatrixTransformation node
  // We will make it read from the proxy_node, which contains the transformation of the haptic device (see pick_example.cpp)
  m_transform_sensor = new osgSensor::TransformSensor(proxy_node);

  // Create a SensorCallback that will read data from a Sensor and update the matrix of a TransformationMatrix node

  m_sensor_callback = new osgSensor::OsgSensorCallback(m_transform_sensor.get());

}

void ButtonEventHandler::buttonDown()
{
  // Does the contacteventhandler has a Shape registrated?
  if (m_cb->getShape())
  {

    m_picked_shape = m_cb->getShape();
    m_cb->getShape()->setEnable(false);

    // Remove the UpdateCallback from the previous moved node
    if (m_last_transform.valid())
      m_last_transform->setUpdateCallback(0L);

    // Get the Transformation node associated with the shape that has collided with the Proxy
    osg::MatrixTransform *t = m_cb->getTransform(m_cb->getShape());
    if (!t) {
      osg::notify(osg::WARN) << "Invalid shape touched, not registrated for moving" << std::endl;
      return;
    }
    osg::notify(osg::WARN) << "Moving shape/node: " << m_cb->getShape()->getName() << std::endl;

    osg::Node *node = t->getChild(0);
    osg::MatrixTransform *mt=0L;

    if (node)
      mt = dynamic_cast<osg::MatrixTransform *>(node);

    if (mt) {
      osg::Matrix mn = t->getMatrix();
      osg::Matrix m1 = mt->getMatrix();
      mn = m1 * mn;
      osg::Matrix mp = m_transform_sensor->getTransform()->getMatrix();

      mp.invert(mp);
      osg::Matrix md = mn*mp;

      mt->setMatrix(md);
    }

    // Enable the sensor
    m_transform_sensor->setEnable(true);

    // set the sensor callback to the newly collided node so we can update its transformation.
    t->setUpdateCallback(m_sensor_callback.get());

    m_last_transform = t;
  }
}

void ButtonEventHandler::buttonUp()
{
  if (m_picked_shape.valid())
    m_picked_shape->setEnable(true);

  // Stop updating the Sensor, so that any node connected via a Updatecallback wont be moved anymore.
  m_transform_sensor->setEnable(false);
}

// Called when any event occurs (button, calibration, update, etc...)
void ButtonEventHandler::operator()(SensorEventHandler::EventType eventType, float time)//push(osgHaptics::EventHandler::Button b)
{
  using namespace osgSensor;
  SensorEventHandler::Button button = getButton();
  SensorEventHandler::ButtonState state = getButtonState();


  if (eventType == osgHaptics::HapticDevice::CALIBRATION_UPDATE_EVENT) {
    std::cerr << "Calibration update" << std::endl;
    return;
  }
  if (eventType == osgHaptics::HapticDevice::CALIBRATION_INPUT_EVENT) {
    std::cerr << "Calibration input" << std::endl;
    return;
  }

  // Break here if its not a button event
  if (eventType != SensorEventHandler::BUTTON)
    return;

  // Is it a button down or up event?
  if (state == SensorEventHandler::DOWN)
    buttonDown();
  else if (state == SensorEventHandler::UP)
    buttonUp();
}


ContactCallback::ContactCallback()
{
  m_default_material = new osg::Material;
  m_highlight_material = new osg::Material;
  m_highlight_material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1,0.2,0.2,1.0));
}


void ContactCallback::contact(osgHaptics::ContactState& state)
{
  // store a pointer to the shape that got in contact with the proxy
  m_contact_shape = state.getShape();

  osg::MatrixTransform *mt = getTransform(m_contact_shape.get());
  if (mt) {
    osg::StateSet *ss = mt->getOrCreateStateSet();
    ss->setAttributeAndModes(m_highlight_material.get());
  }
}

void ContactCallback::separation(osgHaptics::ContactState& state)
{
  
  // reset the contact_shape
  if (!m_contact_shape.valid())
    return;

  osg::MatrixTransform *mt = getTransform(m_contact_shape.get());
  if (mt) {
    osg::StateSet *ss = mt->getOrCreateStateSet();
    ss->setAttributeAndModes(m_default_material.get());
  }

  m_contact_shape = 0L;
}

void PickScene::addNode(osg::MatrixTransform *mt, ButtonEventHandler *button_event_handler, ContactCallback *cb)
{
  m_haptic_root->addChild(mt);
  m_visual_root->addChild(mt);


  std::ostringstream str;
  str << "Shape" << m_n << std::ends;
  osg::ref_ptr<osgHaptics::Shape> shape = new osgHaptics::Shape(m_device.get(), str.str());

  osg::StateSet *ss = mt->getOrCreateStateSet();
  ss->setAttributeAndModes(shape.get());

  cb->addPair(shape.get(), mt);
  osgHaptics::ContactState::ContactEvent mask = osgHaptics::ContactState::ContactEvent(osgHaptics::ContactState::Separation|osgHaptics::ContactState::Contact);

  m_device->registerContactEventHandler(shape.get(), cb, mask);
  
  m_n++;

}


/*
  Create a scene with nodes that can be touched using the proxy.
  When the proxy get in contact and a button is pressed on the device, the object is "picked up", that is following the transformation of the proxy.
*/
PickScene::PickScene(osgHaptics::HapticDevice *device, osg::MatrixTransform *proxy_transform, osg::Group *visual_root, osg::Group *haptic_root) : 
  m_n(0), m_device(device), m_visual_root(visual_root), m_haptic_root(haptic_root)
{


  float radius = 0.8f;
  float height = 1.0f;

  osg::ref_ptr<ContactCallback> cb = new ContactCallback;

  osg::ref_ptr<ButtonEventHandler> button_event_handler = new ButtonEventHandler(cb.get(), proxy_transform);

  osg::TessellationHints* hints = new osg::TessellationHints;
  hints->setDetailRatio(0.5f);
  
  osg::ref_ptr<osg::Group> visual_group = new osg::Group;
  osg::StateSet *ss = visual_group->getOrCreateStateSet();
  ss->setAttributeAndModes(new osgHaptics::Shape(device), osg::StateAttribute::OFF|osg::StateAttribute::PROTECTED|osg::StateAttribute::OVERRIDE);

  device->registerSensorEventHandler(button_event_handler.get());

/*
  osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
  mt->setMatrix(osg::Matrix::translate(0*2,0,0));
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  mt->addChild(geode.get());
  geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius),hints));
  addNode(mt.get(), button_event_handler.get(), cb.get());

  mt = new osg::MatrixTransform;
  mt->setMatrix(osg::Matrix::translate(0*2,0,0));
  geode = new osg::Geode;
  mt->addChild(geode.get());
  geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),radius),hints));
  addNode(mt.get(), button_event_handler.get(), cb.get());

  mt = new osg::MatrixTransform;
  mt->setMatrix(osg::Matrix::translate(0*2,0,0));
  geode = new osg::Geode;
  mt->addChild(geode.get());
  geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),2*radius),hints));
  addNode(mt.get(), button_event_handler.get(), cb.get());

  mt = new osg::MatrixTransform;
  mt->setMatrix(osg::Matrix::translate(0*2,0,0));
  geode = new osg::Geode;
  mt->addChild(geode.get());
  geode->addDrawable(new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0.0f,0.0f,0.0f),radius,height),hints));
  addNode(mt.get(), button_event_handler.get(), cb.get());

  mt = new osg::MatrixTransform;
  mt->setMatrix(osg::Matrix::translate(0*2,0,0));
  osg::ref_ptr<osg::Node> node = osgDB::readNodeFile("cessna.osg.[0.1].scale");
  mt->addChild(node.get());
  addNode(mt.get(), button_event_handler.get(), cb.get());
*/
  
  
  // Add a bunch of geometries
  const int n_rows = 10;
  const int n_cols = 10;
  const int n_z = 10;
  float size = 0.1;
  for(int r=0; r < n_rows; r++)
  {
    for (int c=0; c < n_cols; c++) {
      osg::ref_ptr<osg::MatrixTransform> mt1 = new osg::MatrixTransform;
      mt1->setName("mt1");
      osg::ref_ptr<osg::MatrixTransform> mt2 = new osg::MatrixTransform;
      mt2->addChild(mt1.get());
      mt2->setName("mt2");

      mt2->setMatrix(osg::Matrix::translate(r*size*1.5,c*size*1.5,0));
      osg::ref_ptr<osg::Geode> geode = new osg::Geode;
      mt1->addChild(geode.get());
      geode->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),size),hints));
      addNode(mt2.get(), button_event_handler.get(), cb.get());
    }
  }
}

#if 0

// Called when a button is released
void ButtonEventHandler::release(osgHaptics::EventHandler::Button b)
{

  if (m_picked_shape.valid())
    m_picked_shape->setEnable(true);

  // Stop updating the Sensor, so that any node connected via a Updatecallback wont be moved anymore.
  m_transform_sensor->setEnable(false);
}

#endif