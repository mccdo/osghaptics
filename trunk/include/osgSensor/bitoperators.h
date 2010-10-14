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

#ifndef __osgSensor_bitoperators_h__
#define __osgSensor_bitoperators_h__

namespace osgSensor {

  /// Returns true if bit is set in bitfield
  bool isBitSet(const unsigned int& bitfield, unsigned int bit)  { return ((bitfield>>bit)&01) != 0; } 
  
  /// Returns true if all bits in bitfield is false
  bool isNoBitSet(const unsigned int& bitfield) { return bitfield==0; }

  /// Set the given bit to true
  void setBit(unsigned int& bitfield, unsigned int bit) { bitfield |=  (01 << bit); }

  // Clear the given bit to false
  void clearBit(unsigned int& bitfield, unsigned int bit)  { bitfield &=  ~(01 << bit); }

} // Namespace osgSensor
#endif