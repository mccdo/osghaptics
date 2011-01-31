/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Haptic Library
* Copyright (C) 2006 VRlab, Umeå University
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
*/


#ifndef __osgHaptics_Shape_h__
#define __osgHaptics_Shape_h__

#include <osgHaptics/HapticDevice.h>
#include <osgHaptics/types.h>
#include <osgHaptics/export.h>

#include <HL/hl.h>

#include <osg/StateAttribute>
#include <osg/Referenced>
#include <osg/ref_ptr>
#include <osg/observer_ptr>
#include <osg/Node>



  namespace osgHaptics {

    /// Class that store a  Haptic shape id as a StateAttribute.

    /*!
      This class stores a shape id as a osg StateAttribute.
      When a drawable is rendered in the HapticRenderBin, the shape id of the current
      applied Shape will be used.
    */
    class OSGHAPTICS_EXPORT Shape : public osg::StateAttribute
    {
    public :


      /// Default constructor
      explicit Shape(HapticDevice *device, int enabled=1);

      explicit Shape(HapticDevice *device, const std::string& name);

      //--by SophiaSoo/CUHK: for two arms, New constructor
      explicit Shape();

      /// Enable the haptic rendering of this shape
      virtual void setEnable(bool flag) { m_enabled = flag; }

      /// Return if this shape is enabled for haptic rendering
      bool getEnable() const { return m_enabled ? true : false; }


	  /// Equality operator
    virtual bool operator ==(HLuint shape_id)  {
			int idx = getCurrentDeviceIndex();
			if (isValidIndex(idx)) {
				return shape_id == m_shape_ids[idx];	
			}
			return false;
    }

	  /// Return a pointer to the current haptic device
    HapticDevice *getHapticDevice() {
			int idx = getCurrentDeviceIndex();
			if (isValidIndex(idx)) {
				return m_devices[idx].get();	
			}
			return NULL;
	  }

	  /// Return a pointer to the current haptic device
    const HapticDevice *getHapticDevice() const { 
			int idx = getCurrentDeviceIndex();
			if (isValidIndex(idx)) {
				return m_devices[idx].get();	
			}
			return NULL;
	  }


      /// Set the name of the shape
      void setName(const std::string& name) { m_name = name; }

      /// Return the name of the shape
      std::string getName() const { return m_name; }

      /** Copy constructor using CopyOp to manage deep vs shallow copy. */
      Shape(const Shape& trans,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
        StateAttribute(trans,copyop),
          m_name(trans.m_name),
          m_shape_id(trans.m_shape_id),
          m_enabled(trans.m_enabled),
          m_device(trans.m_device)
          {}


		/// Return the HL shape id of this Shape
    HLint getShapeID() const { 
			int idx = getCurrentDeviceIndex();
			if (isValidIndex(idx)) {
				return m_shape_ids[idx];	
			}
			return 0;
	  }

      //#define META_StateAttribute(library,name,type) 

      virtual osg::Object* cloneType() const { return new Shape(m_device.get(),0); } 
      virtual osg::Object* clone(const osg::CopyOp& copyop) const { return new Shape (*this,copyop); } 
      virtual bool isSameKindAs(const osg::Object* obj) const { return dynamic_cast<const Shape *>(obj)!=NULL; } 
      virtual const char* libraryName() const { return "osgHaptics"; } 
      virtual const char* className() const { return "Shape"; } 
      virtual Type getType() const { return osg::StateAttribute::Type(OSGHAPTICS_SHAPE); }

      //META_StateAttribute(osg, Shape,osg::StateAttribute::Type(OSGHAPTICS_SHAPE));

      /// Get the StateAttribute type of this Shape
      static osg::StateAttribute::Type getSAType()  { return osg::StateAttribute::Type(OSGHAPTICS_SHAPE); }

//--by SophiaSoo/CUHK: for two arms, need change ?????????

      /** Return -1 if *this < *rhs, 0 if *this==*rhs, 1 if *this>*rhs. */
      virtual int compare(const osg::StateAttribute& sa) const
      {
        // Check for equal types, then create the rhs variable
        // used by the COMPARE_StateAttribute_Paramter macros below.
        COMPARE_StateAttribute_Types(Shape,sa)

        // Compare each parameter in turn against the rhs.
        COMPARE_StateAttribute_Parameter(m_shape_id)

        // Compare each parameter in turn against the rhs.
        COMPARE_StateAttribute_Parameter(m_enabled)

        return 0; // Passed all the above comparison macros, so must be equal.
      }

      /*!
        Make this shape id the current one.
        Any openGL primitive will then be drawn into the HL buffer
        Should be called just before any drawable is drawn (but after any modelview/projection matrix is
        modified).
      */
      virtual void postDraw() const; 


      /*!
        Ends the current hl shape
        Should be called just after any drawable is drawn.
      */
      virtual void preDraw() const; 

      virtual bool getModeUsage(ModeUsage& usage) const
      {
        return false;
      }

      /// Apply this StateAttribute. Will actually do nothing as we explicitly will call pre/postDraw()
      virtual void apply(osg::State& state) const {}

      /*!
        During contact, it can be handy to have a node associated to the shape.
        By using setNode()/getNode() a node can be set and later retrieved from a shape.
      */
      void setNode(osg::Node *node) { m_node = node; }
      
      /*!
        Return the associated node
      */
      osg::Node *getNode() { return m_node.get(); }

	  /// Add a device to the list of devices for this shape
	  void addDevice(HapticDevice *device);

		/// Return true if the current device is set
		bool containCurrentDevice() const { return isValidIndex(getCurrentDeviceIndex());	}

    protected :

			
			/// Destructor
			virtual ~Shape();


			/// Return the index for the current device (current in terms of hdGetCurrentDevice)
			int getCurrentDeviceIndex() const;

			/// Return true if the specified index is a valid one
			bool isValidIndex(int index) const { 	bool b = (index<0) ? false : true; return b; }

      std::string m_name;
      HLuint m_shape_id;
      int m_enabled;
      mutable osg::observer_ptr<HapticDevice> m_device;
      //osg::observe_ptr<osg::Node> m_node;
      osg::observer_ptr<osg::Node> m_node;

	  ///--by SophiaSoo/CUHK: for two arms
	  typedef osg::observer_ptr<HapticDevice> Type_ObserverPtr_device;

	  ///--by SophiaSoo/CUHK: for two arms
	  std::vector<Type_ObserverPtr_device> m_devices;

	  ///--by SophiaSoo/CUHK: for two arms
	  std::vector<HLuint> m_shape_ids;

    };

  } // osgHaptics


#endif
