// $Id: StopThread.h,v 1.1 2005/04/11 11:02:17 andersb Exp $

#ifndef __StopThread_h__
#define __StopThread_h__



#include <OpenThreads/Thread>
#include <OpenThreads/ReentrantMutex>
#include <iostream>
#include <OpenThreads/Block>



namespace vrutils {

/// 
/*!
 StopThread contains methods for Stopping it in a nice way.
*/
class StopThread : public OpenThreads::Thread {

private:
  
  bool m_stop;
  OpenThreads::ReentrantMutex m_lock;

	OpenThreads::Block m_killed;

	OpenThreads::ReentrantMutex& getLock() { return m_lock; }
protected:
  bool m_isRunning;


  /*! Called from the Run method to inquiry if anyone has called the Stop method
      returns true when someone has called Stop.
  */
	bool shouldStop(void) { bool b; OpenThreads::ScopedLock<OpenThreads::Mutex> lock(getLock()); b = m_stop; return b; };
  
  /*! Virtual method that must be inherited. This is the main function that
  will be executed in a new thread.
  */
  virtual void run(void) = 0;

  /// Sets the isRunning flag to false. This will cause the isRunning method to return false.
  void exit( void ) { OpenThreads::ScopedLock<OpenThreads::Mutex> lock(getLock()); m_isRunning = false; m_killed.release();  };

public:

  /// Terminates the thread immediately
  //void cancel ( void ) { Thread::cancel(); };
  
  /// Constructor
  StopThread( void ) : m_stop(false), m_isRunning(true) { m_killed.reset();};
  
  /// Destructor
  virtual ~StopThread( void ) { stop(); if (!wait(5)) { cancel(); } };
  
  /// Waits for the thread to die
  bool wait(unsigned long t) { return m_killed.block(t); };

  /// Returns true if the thread is still running
  bool isRunning( void ) { bool b; OpenThreads::ScopedLock<OpenThreads::Mutex> lock(getLock()); b = m_isRunning; return b; };

  /// Stops the thread in a nice way.
  void stop(void) { OpenThreads::ScopedLock<OpenThreads::Mutex> lock(getLock()); m_stop=true;  };

  /*! Checks if the Stop method has been called, if so Exit() method is called and the 
  thread dies in a nice manner
  */
  void Continue( void ) { 
	  if (shouldStop()) 
		exit(); 
	  
  }
}; // Class StopThread
} // Namespace vrutils
#endif

/*------------------------------------------

* $Source: p:\\colosseum3D\\Repository/colosseum3D_1.0/include/vrutils/os/StopThread.h,v $
* $Revision: 1.1 $ 
* $Date: 2005/04/11 11:02:17 $
* $Author: andersb $ 
* $Locker:  $

* Description: 

	VRlab, Umeå University, 2001

* $Log: StopThread.h,v $
--------------------------------------------*/
