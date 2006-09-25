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


#include "dexterity.h"
#include <osg/Group>
#include "DexterityLineDrawable.h"
#include "CatmullRomAnimationPath.h"
#include <osg/Geode>
#include <math.h>
#include <osgHaptics/HapticDevice.h>
#include <osgHaptics/Shape.h>
#include <osgHaptics/Material.h>
#include <osgHaptics/TouchModel.h>
#include <osgHaptics/HapticRenderPrepareVisitor.h>
#include <osgText/Text>
#include <sstream>
#include <osg/CameraNode>
#include <osgHaptics/VibrationForceOperator.h>

double randInterval(double low, double high)
{
  static bool first=true;

  if (first) {
    first = false;
    srand(time(0));
  }
  if (low > high) {
    double t = high;
    high = low;
    low = t;
  }

  double length = high - low; // Length of interval

  double a = rand() / (double)RAND_MAX; // // Rand between 0.0 and 1.0
  double b = length * a + low;  // Rand between low and high

  return b;
}


class TextCallback : public osg::NodeCallback
{
public:
  TextCallback(osgText::Text *text, DexterityLineDrawable *line, osgHaptics::HapticDevice *device, float inner_radius, float outer_radius,
    osgHaptics::ForceOperator *fo) : 
      m_text(text), m_line(line), m_device(device), 
      m_inner_radius(inner_radius), m_outer_radius(outer_radius), m_force_operator(fo) {}

  void operator ()(osg::Node*node, osg::NodeVisitor *nv)
  {
    std::ostringstream str;
    osg::Vec3 pos = m_device->getProxyPosition();
    osg::Vec3 closest;
    float distance = m_line->distance(pos, closest);
    str << "Distance: " << distance;

    if (distance < m_outer_radius && distance > m_inner_radius) {
      m_force_operator->setEnable(true);
      str << " << Touched wire! >>";
    }
    else
      m_force_operator->setEnable(false);
    

      m_text->setText(str.str());
    traverse(node,nv);

  }
private:
  osg::ref_ptr<osgText::Text> m_text;
  osg::ref_ptr<DexterityLineDrawable> m_line;
  osg::ref_ptr<osgHaptics::HapticDevice> m_device;
  osg::ref_ptr<osgHaptics::ForceOperator> m_force_operator;

  float m_inner_radius, m_outer_radius;
};

DexterityLineDrawable *createDexteritySpline(osg::Group *visual_root, osg::Group *haptic_root, osgHaptics::HapticDevice *device)
{
  CatmullRomAnimationPath *path = new CatmullRomAnimationPath;
  osg::AnimationPath::ControlPoint cp;

  int n_points = 10;

  float startx=-2;
  float stopx=2;
  float distx = stopx-startx;
  float deltax = distx/n_points;
  float zampl = 0.5;
  
  float t=0;
  for(int i = 0; i <= n_points; i++, t=i/(float)n_points) {
        
    osg::Vec3 pos;
    pos.set(startx+i*deltax, 0, randInterval(-zampl, zampl));
    cp.setPosition(pos);
    path->insert(t, cp);
  }
  path->build();

  DexterityLineDrawable *line = new DexterityLineDrawable(path);
  line->setResolution(100);

  osg::Vec3 pos;
  line->distance(osg::Vec3(0,0,0), pos);

  osg::Geode *geode = new osg::Geode;
  geode->addDrawable(line);

  visual_root->addChild(geode);
  haptic_root->addChild(geode);

  // Create a haptic material
  osg::ref_ptr<osgHaptics::Material> material = new osgHaptics::Material();

  //Set material attributes
  material->setStiffness(0.8);
  material->setDamping(0.2);
  material->setStaticFriction(0.3);
  material->setDynamicFriction(0.3);

  /*osg::ref_ptr<osgHaptics::TouchModel> touch = new osgHaptics::TouchModel();
  float snap_force_newtons = 5;
  touch->setMode(osgHaptics::TouchModel::CONSTRAINT);
  touch->setSnapDistance(osgHaptics::TouchModel::calcForceToSnapDistance(snap_force_newtons));
  // specify a touch model for this StateAttribute
  ss->setAttributeAndModes(touch.get(), osg::StateAttribute::ON);
  */

  // Prepare the node so that it can be rendered haptically
  // If several nodes are to be prepared, make sure to call the HapticRenderPrepareVisitor::reset() method
  // inbetween
  osgHaptics::HapticRenderPrepareVisitor vis(device);
  geode->accept(vis);

  // Return the compound shape (can be used for contact tests, disabling haptic rendering for this subgraph etc.
  osgHaptics::Shape *shape = vis.getShape();
  osg::StateSet *ss = geode->getOrCreateStateSet();

  // Store the shape.
  //ss->setAttributeAndModes(shape);

  // specify a material attribute for this StateAttribute
  ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON);



  // Create text showing the distance to the line
  osgText::Text *distance_text = new osgText::Text;
  osgText::Font* font = osgText::readFontFile("fonts/arial.ttf");
  if (font)
    distance_text->setFont(font);

  float win_w=1;
  float win_h=1;

  osg::Vec4 layoutColor(1.0f,1.0f,1.0f,1.0f);
  float layoutCharacterSize = 0.05;    
  distance_text->setColor(layoutColor);
  distance_text->setCharacterSize(layoutCharacterSize);
  distance_text->setPosition(osg::Vec3(0.1,0.05,0));
  distance_text->setAxisAlignment(osgText::Text::SCREEN);
  distance_text->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

  {

    float outer_radius = 0.2;
    float inner_radius = 0.15;
    osgHaptics::VibrationForceOperator *force_operator = new osgHaptics::VibrationForceOperator();
    device->addForceOperator(force_operator);

    force_operator->setFrequency(50);
    force_operator->setDirection(osg::Vec3d(0,0,1));
    force_operator->setAmplitude(1.3);

    osg::Geode *geode = new osg::Geode;
    geode->addDrawable(distance_text);
    geode->setUpdateCallback(new TextCallback(distance_text, line, device, 
                                              inner_radius, outer_radius, 
                                              force_operator));

    
    // create the hud.
    osg::CameraNode* camera = new osg::CameraNode;
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setProjectionMatrixAsOrtho2D(0,win_w,0,win_h);
    camera->setViewMatrix(osg::Matrix::identity());
    camera->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);

    visual_root->addChild(camera);
    camera->addChild(geode);


  }

  return line;
}
