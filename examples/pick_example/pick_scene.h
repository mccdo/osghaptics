#ifndef _Pick_Scene_h__
#define _Pick_Scene_h__


#include <osg/Group>
#include <osg/Material>
#include <osgHaptics/HapticDevice.h>
#include <osg/MatrixTransform>
//#include <osgHaptics/EventHandler.h>
#include <osgHaptics/Shape.h>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osgSensor/TransformSensor.h>
#include <osgSensor/OsgSensorCallback.h>
#include <osg/Notify>
#include <osgDB/readfile>
#include <osgHaptics/ContactEventHandler.h>




/// Class that will react to contact between a registrated shape and the proxy.
class ContactCallback : public osgHaptics::ContactEventHandler {
public:
  ContactCallback();

  virtual void contact( osgHaptics::ContactState& state);
  virtual void separation( osgHaptics::ContactState& state);

  /// Return the shape that was last in contact
  osgHaptics::Shape *getShape() { return m_contact_shape.get(); }

  /// Associate a shape with a MatrixTransformation node.
  void addPair(osgHaptics::Shape *shape, osg::MatrixTransform *node) { m_shape_map[shape] = node; }

  /// Return a MatrixTransformation that earlier has been associated with a Shape
  osg::MatrixTransform *getTransform(osgHaptics::Shape *shape) { 
    ShapeMap::iterator it = m_shape_map.find(shape);
    if (it != m_shape_map.end())
      return it->second;

    return 0L;
  }


protected:

  typedef std::map<osgHaptics::Shape *, osg::MatrixTransform *> ShapeMap;
  ShapeMap m_shape_map;

  virtual ~ContactCallback() {}
  osg::ref_ptr<osgHaptics::Shape> m_contact_shape;
  osg::ref_ptr<osg::Material> m_default_material, m_highlight_material;


};

/// Class that will react to button press/release
class ButtonEventHandler : public osgSensor::SensorEventHandler
{
public:

  ButtonEventHandler(ContactCallback *cb, osg::MatrixTransform *proxy_node);


  /// Virtual method that will be called upon a button press/release
  virtual void operator()(EventType eventType, float time);

//  virtual void push(osgHaptics::EventHandler::Button b);

  /// Virtual method that will be called upon a button release
  //virtual void release(osgHaptics::EventHandler::Button b);



protected:

  void buttonDown();
  void buttonUp();

  osg::ref_ptr<ContactCallback> m_cb;
  osg::ref_ptr<osgSensor::TransformSensor> m_transform_sensor;
  osg::ref_ptr<osgSensor::OsgSensorCallback> m_sensor_callback;

  osg::ref_ptr<osgHaptics::Shape> m_picked_shape;
  osg::ref_ptr<osg::MatrixTransform> m_last_transform;

};



class PickScene : public osg::Referenced {
public:
  PickScene(osgHaptics::HapticDevice *device, osg::MatrixTransform *proxy_transform, osg::Group *visual_root, osg::Group *haptic_root);

  void addNode(osg::MatrixTransform *mt, ButtonEventHandler *button_event_handler, ContactCallback *cb);

private:

  osg::ref_ptr<osg::Group> m_haptic_root, m_visual_root;
  osg::ref_ptr<osgHaptics::HapticDevice> m_device;
  unsigned int m_n;

};
  

#endif
