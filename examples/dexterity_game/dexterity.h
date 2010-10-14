#ifndef _DEXTERITY_
#define _DEXTERITY_

#include <osg/Group>
#include <osgHaptics/HapticDevice.h>
#include "DexterityLineDrawable.h"

DexterityLineDrawable *createDexteritySpline(osg::Group *visual_root, osg::Group *haptic_root, osgHaptics::HapticDevice *device);

#endif

