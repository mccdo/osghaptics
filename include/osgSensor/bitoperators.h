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