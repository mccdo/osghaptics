#ifndef __HapticMaterialVisitor_h__
#define __HapticMaterialVisitor_h__

#include <osg/NodeVisitor>
#include <osg/Node>
#include <osgHaptics/HapticDevice.h>

/// Class that extracts Haptic information from Description strings attached to nodes in the scene
/*!

  Valid description strings are:

  Shape <anything>
  Material <anything>
    DynamicFriction <float> [0..1]
    StaticFriction <float> [0..1]
    Stiffness <float> [0..)
    Damping   <float> [0..)
  TouchModel CONTACT/CONSTRAINT
    SnapForce <float> [0..)

*/
class HapticMaterialVisitor : public osg::NodeVisitor{
public:
  
  /// Constructor
  HapticMaterialVisitor(osgHaptics::HapticDevice *device);

  virtual void apply(osg::Node& node);

private:
  osg::ref_ptr<osgHaptics::HapticDevice> m_device;
};

#endif
