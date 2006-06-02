
#include  <osgHaptics/HashedGrid.h>
#include  <osgHaptics/HashedGridDrawable.h>
#include  <osgHaptics/TriangleExtractor.h>

#include <osg/ref_ptr>
#include <time.h>
#include <iostream>
#include <osg/io_utils>

class  HashGridTriangleExtractOperator : public osgHaptics::TriangleExtractOperator
{

public:

  /// Constructor
  HashGridTriangleExtractOperator(osgHaptics::HashedGridDrawable::TriangleHashGrid *grid) : m_hash_grid(grid) { }

  /*!
  This method is a pure virtual method that has to be inherited. 
  This method will be called for each triangle. The vertices are given in
  world coordinates using the accumulated matrix.
  */
  virtual void triangle(const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3)
  {
    osgHaptics::HashedGridDrawable::Triangle *triangle = new osgHaptics::HashedGridDrawable::Triangle(v1,v2,v3);
    m_hash_grid->insert(v1, triangle, triangle);
    m_hash_grid->insert(v2, triangle, triangle);
    m_hash_grid->insert(v3, triangle, triangle);
  }


private:
  osg::ref_ptr<osgHaptics::HashedGridDrawable::TriangleHashGrid> m_hash_grid;
};



#if 0

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

struct Triangle : osg::Referenced {
  Triangle(const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3): m_p1(p1), m_p2(p2), m_p3(p3) {}
  osg::Vec3 m_p1, m_p2, m_p3;
//  virtual ~Triangle() { std::cerr << this << std::endl; }
};



void main()
{
  using namespace osgHaptics;
  osg::Vec3 bounds;
  osg::ref_ptr<HashedGridDrawable::TriangleHashGrid> grid = new HashedGridDrawable::TriangleHashGrid(bounds, 10);

  HashGridTriangleExtractOperator hteo(grid.get());
  TriangleExtractor te(hteo);
  osg::ref_ptr<HashedGridDrawable> grid_drawable = new HashedGridDrawable(grid.get(), 0L);
  osg::Geode *geode = new osg::Geode;
  geode->addDrawable(grid_drawable.get());

/*  typedef osgHaptics::HashedGrid< osg::ref_ptr<Triangle> > HashedGridTriangle;
  HashedGridTriangle grid(osg::Vec3(10,10,10), 10);
  grid.setCenter(osg::Vec3(0,0,0));
  
  srand(234);
  for (unsigned int i=0; i < 2000; i++) {
    
    float x = randInterval(-10, 10);
    float y = randInterval(-10, 10);
    float z = randInterval(-10, 10);
    float f = (float)i;
    osg::Vec3 p1(x,y,z);
    Triangle *t = new Triangle(p1,p1,p1);
    grid.insert(osg::Vec3(x,y,z), t);
  }

  HashedGridTriangle::iterator it = grid.begin();
  for(; it != grid.end(); it++) {
    std::cerr << "size: " << (*it).size() << std::endl;

  }

  osg::Vec3 p;
  HashedGridTriangle::const_reference cit = grid.intersect(p);

  std::cerr << "There are " << cit.size() << " triangles in proximity to position " << p << std::endl;
  
  grid.clear();
  */
}

#else

/* -*-c++-*- OpenSceneGraph Haptics Library - * Copyright (C) 2006 VRlab, Ume� University
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

using namespace sensors;

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

    void setColor(float r, float g, float b ) { m_color.set(r,g,b); }

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

// create contact callback
class  MyContactEventHandler : public osgHaptics::ContactEventHandler
{
public:
  /// Base constructor
  MyContactEventHandler(){};

  // Called upon contact between proxy and a specified shape
  virtual void contact( osgHaptics::ContactState& state){ osg::notify(osg::WARN) << "contact" << state << std::endl; };

  // Called upon separation between proxy and a specified shape
  virtual void separation( osgHaptics::ContactState& state){ osg::notify(osg::WARN) << "separation" << std::endl; };

  // Called when proxy is moved in contact with a specified shape
  virtual void motion( osgHaptics::ContactState& state){  osg::notify(osg::INFO) << "motion" << state << std::endl; };

protected:


  /// Destructor
  virtual ~MyContactEventHandler() {}

};




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
  arguments.getApplicationUsage()->addCommandLineOption("--constraint <distance>","Set the material TouchMode to Constraint with specified distance");
  arguments.getApplicationUsage()->addCommandLineOption("--proxy-scale <scale>","Set the size of the proxy pen");
  arguments.getApplicationUsage()->addCommandLineOption("--dynamic-friction <float>","Set the dynamic friction of the haptic surface");
  arguments.getApplicationUsage()->addCommandLineOption("--static-friction <float>","Set the static friction of the haptic surface");
  arguments.getApplicationUsage()->addCommandLineOption("--stiffness <float>","Set the stiffness in the friction equation (spring)");
  arguments.getApplicationUsage()->addCommandLineOption("--damping <float>","Set the damping in the friction equation (spring)");
  arguments.getApplicationUsage()->addCommandLineOption("--hash ","Hash the loaded model for intersection test");
  arguments.getApplicationUsage()->addCommandLineOption("--cell-size", "Size of the hash cell");
  arguments.getApplicationUsage()->addCommandLineOption("--render-triangles", "Do not render the triangles that are intersected with the proxy");
  arguments.getApplicationUsage()->addCommandLineOption("--remove-instances","Do a deep copy and remove any instances of the haptic scenegraph");


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

  // See if scale for proxy pen is specified as an argument
  float proxy_scale=1.0f;
  bool set_proxy_scale = arguments.read("--proxy-scale", proxy_scale);

  // See if constraint mode is specified for the surface
  float constraint_force_newton=0;
  bool use_constraint = arguments.read("--constraint", constraint_force_newton);

  bool use_hash = arguments.read("--hash");

  int render_hash_triangles=0;
  arguments.read("--render-triangles", render_hash_triangles);

  int cell_size=10;
  arguments.read("--cell-size", cell_size);
  if (use_hash)
    std::cerr << "Cell size: " << cell_size << std::endl;

  // See if dynamic_friction is specified
  float dynamic_friction=0.5;
  bool set_dynamic_friction = arguments.read("--dynamic-friction", dynamic_friction);

  // See if static_friction is specified
  float static_friction=0.5;
  bool set_static_friction = arguments.read("--static-friction", static_friction);

  // See if damping is specified
  float damping=0.5;
  bool set_damping = arguments.read("--damping", damping);

  // See if stiffness is specified
  float stiffness=0.5;
  bool set_stiffness = arguments.read("--stiffness", stiffness);

  bool remove_instances = arguments.read("--remove-instances");

  // report any errors if they have occured when parsing the program arguments.
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
    return 1;
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
    haptic_device->init(); // Initialize it
    haptic_device->makeCurrent(); // Make this device the current one
    haptic_device->setEnableForceOutput(true); // Render output forces


    // Root of the haptic scene
    osg::ref_ptr<osgHaptics::HapticRootNode> haptic_root = new osgHaptics::HapticRootNode(&viewer);
    root->addChild(haptic_root.get());


    // Create a haptic material
    osg::ref_ptr<osgHaptics::Material> material = new osgHaptics::Material();

    //Set material attributes
    if (set_stiffness)
      material->setStiffness(stiffness);
    else
      material->setStiffness(0.8);

    if (set_damping)
      material->setDamping(damping);
    else
    material->setDamping(0.2);

    if (set_static_friction)
      material->setStaticFriction(static_friction);
    else
      material->setStaticFriction(0.7);

    if (set_dynamic_friction)
      material->setDynamicFriction(dynamic_friction);
    else
      material->setDynamicFriction(0.3);


    // Create a visitor that will prepare the Drawables in the subgraph so they can be rendered haptically
    // It merely attaches a osgHaptics::Shape ontop of each Drawable.
    // 

    if (remove_instances) {
      std::cerr << "Removing instances" << std::endl;
      osg::Object *clone = loadedModel->clone(osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL));
      osg::ref_ptr<osg::Node> node = dynamic_cast<osg::Node *>(clone);
      loadedModel = node;
    }

    osg::Node *visual_node = loadedModel.get();
    osg::Node *haptic_node = loadedModel.get();
  
    if (use_hash)
    {
      std::cerr << "Will hash model " << std::endl;
      using namespace osgHaptics;
      osg::Vec3 dim;

      osg::BoundingSphere bs = loadedModel->getBound();
      osg::BoundingBox bbox;
      bbox.expandBy(bs);

      dim = bbox._max - bbox._min;
      osg::Vec3 center = bbox.center();
      osg::ref_ptr<osgHaptics::HashedGridDrawable::TriangleHashGrid> grid = new osgHaptics::HashedGridDrawable::TriangleHashGrid(dim, cell_size);
      grid->setCenter(center);

      HashGridTriangleExtractOperator hteo(grid.get());
    
      TriangleExtractor te(hteo);

      osg::Timer_t start = osg::Timer::instance()->tick();
      loadedModel->accept(te);
      osg::Timer_t stop = osg::Timer::instance()->tick();
      std::cerr << "Time to hash " << "  t: " << osg::Timer::instance()->delta_m(start,stop) << std::endl;
      
      osg::ref_ptr<osgHaptics::HashedGridDrawable> grid_drawable = new osgHaptics::HashedGridDrawable(grid.get(), haptic_device.get());

      osg::Geode *geode = new osg::Geode;
      geode->addDrawable(grid_drawable.get());

      if (render_hash_triangles)
        visual_root->addChild(geode);

      haptic_node = geode;
    }

    osgHaptics::HapticRenderPrepareVisitor vis(haptic_device.get());
    haptic_node->accept(vis);
    
    loadedModel->accept(vis);


    // Return the compound shape (can be used for contact tests, disabling haptic rendering for this subgraph etc.
    osgHaptics::Shape *shape = vis.getShape();
    osg::StateSet *ss = haptic_node->getOrCreateStateSet();

    // Store the shape.
    ss->setAttributeAndModes(shape);

    osg::ref_ptr<osgHaptics::TouchModel> touch = new osgHaptics::TouchModel();
    
    if (use_constraint)
      touch->setMode(osgHaptics::TouchModel::CONSTRAINT);
    else
      touch->setMode(osgHaptics::TouchModel::CONTACT);

    touch->setSnapDistance(osgHaptics::TouchModel::calcForceToSnapDistance(constraint_force_newton));


    // specify a material attribute for this StateAttribute
    ss->setAttributeAndModes(material.get(), osg::StateAttribute::ON);

    // specify a touch model for this StateAttribute
    ss->setAttributeAndModes(touch.get(), osg::StateAttribute::ON);


    // add it to the visual node to be rendered visually
    visual_root->addChild(visual_node);

    // Add it to the haptic root
    haptic_root->addChild(haptic_node);

    // Add a contact handler
    // Whenever contact/separation/movement of the proxy onto the shape occurs, the callbackclass will
    // be triggered
    haptic_device->registerContactEventHandler(shape, 
      new MyContactEventHandler, 
      osgHaptics::ContactState::All); // React on contact+separation+movement


    // Create a proxy sphere for rendering
    std::ostringstream str;
    if (set_proxy_scale)
      str << "pen.ac.[" << proxy_scale << "].scale";
    else
      str << "pen.ac";

    osg::ref_ptr<osg::Node> proxy_sphere = osgDB::readNodeFile(str.str()); // Load a pen geometry
    osg::ref_ptr<osg::MatrixTransform> proxy_transform = new osg::MatrixTransform;    
    proxy_transform->addChild(proxy_sphere.get());

    visual_root->addChild(proxy_transform.get());

    /*
    Add pre and post draw callbacks to the camera so that we start and stop a haptic frame
    at a time when we have a valid OpenGL context.
    */
    osgHaptics::prepareHapticCamera(&viewer, haptic_device.get(), 0,root.get());

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
    haptic_device->setWorkspaceMode(osgHaptics::HapticDevice::VIEW_MODE);

    double hfov = viewer.getCamera(0)->getLens()->getHorizontalFov();
    double vfov = viewer.getCamera(0)->getLens()->getVerticalFov();
    //viewer.getCamera(0)->getLens()->setPerspective( hfov, vfov, 1, 10 );

    int camera_no = 0;
    double left, right, bottom, top, nearclip, farclip;
    /*viewer.getCamera(camera_no)->getLens()->getParams(left, right, bottom, top, nearclip, farclip);
    viewer.getCamera(camera_no)->getLens()->setAutoAspect(false); 
    viewer.getCamera(camera_no)->getLens()->setFrustum(left, right, bottom, top, 1, 1.2*radius); 
    */
    //viewer.getCamera(0)->getLens()->setPerspective( hfov, vfov, 1, bs.radius()*3 );

    viewer.getCamera(camera_no)->getLens()->getParams(left, right, bottom, top, nearclip, farclip);

    osg::BoundingBox bbox;
    bbox.expandBy(bs);


    //osg::Vec3 mmin(-0.5,-0.5, -0.5);
    //osg::Vec3 mmax(0.5,0.5, 0.5);

    /*
    Another way is to set the workspace of the Haptic working area to enclose the bounding box of the scene.
    Also, set the WorkSpace mode to use the Bounding box
    Notice that the haptic device will not follow the camera as it does by default. (VIEW_MODE)
    */
    haptic_device->setWorkspace(bbox._min, bbox._max);
    //haptic_device->setWorkspaceMode(osgHaptics::HapticDevice::BBOX_MODE);


    /* 
    A transformation using the TouchWorkSpace matrix will also scale all the force/position and orientation output
    from the haptic device.

    It will also affect the size, orientation and position of the haptic workspace.
    Practical if you want to rotate the workspace relative to the device.
    */
    osg::Matrixd tm;
    /*tm.makeRotate(osg::PI_4, osg::Vec3(1,0,0), 
    osg::PI_4*0.4, osg::Vec3(1,0,0), 
    osg::PI_4*1.4, osg::Vec3(1,0,0));*/
    //tm.makeScale(2,2,2);

    haptic_device->setTouchWorkspaceMatrix(tm);
    osg::ref_ptr<osgSensor::OsgSensor> sensor = new osgSensor::OsgSensor(haptic_device.get());
    osg::ref_ptr<osgSensor::OsgSensorCallback> sensor_callback = new osgSensor::OsgSensorCallback(sensor.get());

    proxy_transform->setUpdateCallback(sensor_callback.get());

    bool set_pos = false;


    // Add a custom drawable that draws the rendered force of the device
    VectorDrawable *force_drawable;
    {
      osg::Geode *geode = new osg::Geode;
      force_drawable = new VectorDrawable();
      geode->addDrawable(force_drawable);
      visual_root->addChild(geode);
    }

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

    // Shutdown all registrated sensors
    osgSensor::SensorMgr::instance()->shutdown();

  } catch (std::exception& e) {
    osg::notify(osg::FATAL) << "Caught exception: " << e.what() << std::endl;
  }

  return 0;
}
#endif