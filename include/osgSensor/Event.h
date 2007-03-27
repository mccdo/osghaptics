/* -*-c++-*- $Id: Version,v 1.2 2004/04/20 12:26:04 andersb Exp $ */
/**
* OsgHaptics - OpenSceneGraph Haptic Library
* Copyright (C) 2006 VRlab, Ume� University
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

#ifndef _osgsensor_EVENT_
#define _osgsensor_EVENT_

#include <OsgSensor/export.h>
#include <OpenThreads/ReentrantMutex>
#include <OpenThreads/Condition>


/// Class that implements event notification. One or more threads can signal a waiting event.
/*!
  This class implements a feature available in Win32API. EventNotification.
  It can hold one thread waiting for a resource, one or more other threads can signal and release this event.
  A timeout can specify the amount of time that a thread should wait for this event to be signalled.
*/
namespace osgSensor {

class OSGSENSOR_EXPORT Event
{
  enum {
    TIMEOUT_INFINITE=
#ifndef _WIN32
      INFINITE         
#else
      0xFFFFFFFF  // Infinite timeout
#endif
 
  };
private:

	OpenThreads::ReentrantMutex m_mutex;
  OpenThreads::Condition m_condition;

  bool m_signaled;
  int m_count;

public:
	
  /// Default constructor
  Event();

  /// Destructor 
	virtual ~Event();

	/**
	 * When an Event has been signaled, it has to be reset before it can respond to a new signal.
	 * 
	 */
	void reset(void);

	/**
	 * Signal the event for the waiting thread.
	 */
	void signal(void);

	/**
	 * Wait either for the event to be signaled by another thread or
	 * for the specified timeout duration.
	 * 
	 * @return true if it has been signaled, fase if it didnt get a signal before timeout.
	 * @param timer timeout in milliseconds to wait for a signal.
	 */
	bool wait(unsigned long timeout=INFINITE);

private:

  /**
  *  Private copy constructor, to prevent tampering.
  */
  Event(const Event &/*c*/) {};

  /**
  *  Private copy assignment, to prevent tampering.
  */
  Event &operator=(const Event &/*c*/) {return *(this);};

};


}

#endif
