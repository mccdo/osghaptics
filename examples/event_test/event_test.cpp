#include <osgSensor/Event.h>
#include <iostream>

#include <OpenThreads/Thread>

class MyThread : public OpenThreads::Thread
{
public:
  MyThread(osgSensor::Event *event) : m_event(event) {}

  void run()
  {
    while(1) {
      std::cerr << "Thread: " << this << "In run, waiting for event to signal" << std::endl;
      if (!m_event->wait(1)) {
        std::cerr << "Timeout waiting" << std::endl;
      }
      std::cerr << "Thread: " << this << "In run, Event is signalled" << std::endl;
      microSleep( 1000 * 1000 );
    }

  }
private:
  osgSensor::Event *m_event;
};

void main()
{
  osgSensor::Event event;

  MyThread thread1(&event);

  thread1.start();

  Sleep(1000);
  event.signal();
  event.reset();
  Sleep(3000);
  std::cerr << "Finishing main" << std::endl;
  thread1.cancel();
  OpenThreads::Thread::microSleep(1000*1);
}
