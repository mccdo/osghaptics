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

#ifndef __osghaptics_export_h__
#define __osghaptics_export_h__

#ifdef _WIN32

#pragma once


// Windows Header Files:
//#include <windows.h>

#ifdef OSGHAPTICS_EXPORTS
#define OSGHAPTICS_EXPORT __declspec(dllexport)
#else
#define OSGHAPTICS_EXPORT __declspec(dllimport)
#endif

// This class is exported from the osgHaptics.dll
class OSGHAPTICS_EXPORT CosgHaptics {
public:
	CosgHaptics(void);
	// TODO: add your methods here.
};

extern OSGHAPTICS_EXPORT int nosgHaptics;

OSGHAPTICS_EXPORT int fnosgHaptics(void);


#else // _WIN32
#define OSGHAPTICS_EXPORT
#endif


#endif
