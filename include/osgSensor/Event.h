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

#ifndef _osgsensor_EVENT_
#define _osgsensor_EVENT_

#include <OsgSensor/export.h>
#include <OpenThreads/Mutex>
#include <OpenThreads/Condition>



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

  OpenThreads::Mutex m_mutex;
  OpenThreads::Condition m_condition;

  bool m_signaled;
  int m_count;

public:
	Event();

	virtual ~Event();

	/**
	 * Once signaled, the Event class must be "reset" before responding
	 * to a new signal.
	 * 
	 * @see #signal
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
	 * @see #signal
	 * @return true if signaled, false if timed out.
	 * @param timer timeout in milliseconds to wait for a signal.
	 */
	bool wait(unsigned long timeout);
	bool wait(void);

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
