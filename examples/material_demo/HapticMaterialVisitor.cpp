#include "HapticMaterialVisitor.h"
#include <osgHaptics/Shape.h>
#include "string_utils.h"
#include <osgHaptics/HapticRenderPrepareVisitor.h>
#include <osgHaptics/Material.h>
#include <osgHaptics/TouchModel.h>

HapticMaterialVisitor::HapticMaterialVisitor(osgHaptics::HapticDevice *device) : 
  NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_device(device)
{
}

void HapticMaterialVisitor::apply(osg::Node& node)

{
  // Some default values.
  float static_friction = 1.0;
  float dynamic_friction = 1.0;
  float stiffness = 1.0;
  float damping = 0.2;
  float contact = 1.0;
  
  osg::ref_ptr<osgHaptics::Material> material;
  osg::ref_ptr<osgHaptics::TouchModel> touch;

  // Is there any descriptions attached to this node?
  unsigned int n = node.getNumDescriptions();
  if (!n) {
    traverse(node);
    return;
  }

  osg::notify(osg::WARN) << "Found node with description: " << node.getName() << std::endl;


  // Iterate over all descriptions
  const osg::Node::DescriptionList &list = node.getDescriptions();
  osg::Node::DescriptionList::const_iterator it = list.begin();
  for(; it != list.end(); it++) {
    std::string str = *it;
    std::string key, s_data;
    float f_data;
    
    bool float_data = getData(str, key, s_data, f_data);

    // Should we create a Shape?
    if (!cmp_nocase(key, "Shape")) {
      // Create a visitor that will prepare the Drawables in the subgraph so they can be rendered haptically
      // It merely attaches a osgHaptics::Shape ontop of each Drawable.
      // 
      osg::notify(osg::WARN) << "  Found Shape" << std::endl;
      osgHaptics::HapticRenderPrepareVisitor vis(m_device.get());
      node.accept(vis);

      // Return the compound shape (can be used for contact tests, disabling haptic rendering for this subgraph etc.
      osgHaptics::Shape *shape = vis.getShape();
      osg::StateSet *ss = node.getOrCreateStateSet();
      ss->setAttributeAndModes(shape);

    }
    
    // Should we create a Material?
    else if (!cmp_nocase(key, "Material")) {
      osg::notify(osg::WARN) << "  Found Material" << std::endl;
      material = new osgHaptics::Material();
    }
    else if (!cmp_nocase(key, "StaticFriction")) {
      if (material.valid()) {
        if (!float_data) {
          osg::notify(osg::WARN) << "Found StaticFriction, but it is not of float type in node " << node.getName() << std::endl;            
          traverse(node);

          return;
        }
        osg::notify(osg::WARN) << "    StaticFiction: " << f_data << std::endl;
        material->setStaticFriction(f_data);
      } else {
        osg::notify(osg::WARN) << "Found StaticFriction, but missing an earlier Material key in node " << node.getName() << std::endl;
        traverse(node);

        return;
      }
    }
    else if (!cmp_nocase(key, "DynamicFriction")) {
      if (material.valid()) {
        if (!float_data) {
          osg::notify(osg::WARN) << "Found DynamicFriction, but it is not of float type in node " << node.getName() << std::endl;            
          traverse(node);

          return;
        }
        osg::notify(osg::WARN) << "    DynamicFiction: " << f_data << std::endl;
        material->setDynamicFriction(f_data);
      } else {
        osg::notify(osg::WARN) << "Found DynamicFriction, but missing an earlier Material key in node " << node.getName() << std::endl;
        traverse(node);
        return;
      }
    }
    else if (!cmp_nocase(key, "Stiffness")) {
      if (material.valid()) {
        if (!float_data) {
          osg::notify(osg::WARN) << "Found Stiffness, but it is not of float type in node " << node.getName() << std::endl;            
          traverse(node);

          return;
        }
        osg::notify(osg::WARN) << "    Stiffness: " << f_data << std::endl;
        material->setStiffness(f_data);
      } else {
        osg::notify(osg::WARN) << "Found Stiffness, but missing an earlier Material key in node " << node.getName() << std::endl;
        traverse(node);

        return;
      }
    }
    else if (!cmp_nocase(key, "Damping")) {
      if (material.valid()) {
        if (!float_data) {
          osg::notify(osg::WARN) << "Found Damping, but it is not of float type in node " << node.getName() << std::endl;            
          traverse(node);
          return;
        }
        osg::notify(osg::WARN) << "    Damping: " << f_data << std::endl;
        material->setDamping(f_data);
      } else {
        osg::notify(osg::WARN) << "Found Damping, but missing an earlier Material key in node " << node.getName() << std::endl;
        traverse(node);

        return;
      }
    }
    
    else if (!cmp_nocase(key, "TouchModel")) {
      osg::notify(osg::WARN) << "Found TouchModel" << std::endl;
      touch = new osgHaptics::TouchModel();
   
      if (!cmp_nocase(s_data, "CONSTRAINT")) {
        touch->setMode(osgHaptics::TouchModel::CONSTRAINT);
        osg::notify(osg::WARN) << "  TouchModel: CONSTRAINT"  << std::endl;
      }
      else if (!cmp_nocase(s_data, "CONTACT")) {
        touch->setMode(osgHaptics::TouchModel::CONTACT);
        osg::notify(osg::WARN) << "  TouchModel: CONTACT"  << std::endl;
      }
      else {
        osg::notify(osg::WARN) << "Unknown TouchModel (" << s_data << ") specified in node " << node.getName() << std::endl;

      }
    }
    else if (!cmp_nocase(key, "SnapForce")) {
      if (touch.valid()) {
        if (!float_data) {
          osg::notify(osg::WARN) << "Found SnapForce, but it is not of float type in node " << node.getName() << std::endl;            
          traverse(node);
          return;
        }

        osg::notify(osg::WARN) << "    SnapForce: " << f_data << std::endl;
        touch->setSnapDistance(osgHaptics::TouchModel::calcForceToSnapDistance(f_data));       

      } else {
        osg::notify(osg::WARN) << "Found SnapForce, but missing an earlier TouchModel key in node " << node.getName() << std::endl;
        traverse(node);

        return;
      }
    }

  } // for


  // If a material has been created, attach it to the StateSet
  if (material.valid()) {
    node.getOrCreateStateSet()->setAttributeAndModes(material.get(),osg::StateAttribute::ON);
  }

// If a touch has been created, attach it to the StateSet
  if (touch.valid()) {
    node.getOrCreateStateSet()->setAttributeAndModes(touch.get(),osg::StateAttribute::ON);
  }

  traverse(node);
}

