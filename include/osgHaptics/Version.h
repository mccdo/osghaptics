#ifndef OSGHAPTICS_VERSION
#define OSGHAPTICS_VERSION 1

#include <osgHaptics/export.h>

extern "C" {

  /**
  * osgHapticsGetVersion() returns the library version number.
 *
  * This C function can be also used to check for the existence of the OpenHaptics
  * library using autoconf and its m4 macro AC_CHECK_LIB.
  *
  * Here is the code to add to your configure.in:
  \verbatim
  #
  # Check for the OpenHaptics library
  #
  AC_CHECK_LIB(osg, osgHaptics, ,
  [AC_MSG_ERROR(OpenHaptics library not found.)],)
  \endverbatim
  */
  extern OSGHAPTICS_EXPORT const char* osgHapticsGetVersion();

  /**
  * osgHapticsGetLibraryName() returns the library name in human friendly form.
  */
  extern OSGHAPTICS_EXPORT const char* osgHapticsGetLibraryName();

}

#endif
