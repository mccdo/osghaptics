/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Sensor Library
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

#include <osgSensor/KeyboardSensor.h>
#include <osg/Notify>

using namespace osgSensor;

KeyboardSensor::KeyboardSensor() : OsgSensor()
{
}

void KeyboardSensor::sample()
{
  calcMatrix();
}

bool KeyboardSensor::handle(int key)
{
  bool handled = false;
  float inc = 0.01;
  switch(key) 
    {
      case 'x':
        {
          osg::notify(osg::INFO) << "handle" << "Pressed x" << std::endl;
          osg::Quat q;
          q.makeRotate(inc, 0, 1, 0);
//          q.makeRotate(0, inc, 0);
          q  = getRotation() * q;
          setRotation(q);

          handled = true;

        }
        break;
      case 'X':
        {
          osg::notify(osg::INFO) <<  "Pressed X"<< std::endl;

          osg::Quat q;
          q.makeRotate(-inc, 0, 1, 0);
//          q.makeRotate(0, -inc, 0);
          q  = getRotation() * q;
          setRotation(q);
          handled = true;
        }
        break;

      case 'y':
        {
          osg::notify(osg::INFO) <<  "Pressed y" << std::endl;
          osg::Quat q;
          q.makeRotate(inc, 0, 0, 1);
//          q.makeRotate(0, 0, inc);
          q  = getRotation() * q;
          setRotation(q);
          handled = true;
        }
        break;
    case 'Y':
        {
          osg::notify(osg::INFO) <<  "Pressed Y" << std::endl;
          osg::Quat q;
          q.makeRotate(-inc, 0, 0, 1);
//          q.makeRotate(0, 0, -inc);
          q  = getRotation() * q;
          setRotation(q);
          handled = true;
        }
        break;

    case 'z':
        {
          osg::notify(osg::INFO) << "Pressed z" << std::endl;
          osg::Quat q;
          q.makeRotate(inc, 1, 0, 0);
//          q.makeRotate(inc, 0, 0);
          q  = getRotation() * q;
          setRotation(q);
          handled = true;
        }
        break;
    case 'Z':
        {
          osg::notify(osg::INFO) << "Pressed Z" << std::endl;
          osg::Quat
          q;
          q.makeRotate(-inc, 1, 0, 0);
          //q.makeRotate(-inc, 0, 0);
          q  = getRotation() * q;
          setRotation(q);
          handled = true;
        }
        break;


      default:
        break;

    }
  calcMatrix();
  return handled;
}

