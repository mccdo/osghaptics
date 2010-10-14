#include <OpenThreads/Block>
#include <iostream>

#include <OpenThreads/Thread>

class MyThread : public OpenThreads::Thread
{
public:
  MyThread(OpenThreads::Block *block) : m_event(block) {}

  void run()
  {
    while(1) {
      std::cerr << "Thread: " << this << "In run, waiting for event to signal" << std::endl;
      if (!m_event->block(999)) {
        std::cerr << "Timeout waiting" << std::endl;
      }
			else {
				std::cerr << "Thread: " << this << "In run, Block is signalled" << std::endl;
				microSleep( 1000 * 1000 );
			}
    }

  }
private:
  OpenThreads::Block *m_event;
};

void main()
{
  OpenThreads::Block block;

  MyThread thread1(&block);

  thread1.start();

	OpenThreads::Thread::microSleep(1000*1000);
  block.release();
	OpenThreads::Thread::microSleep(1000*1000);
  block.reset();
	OpenThreads::Thread::microSleep(1000*1000);
  std::cerr << "Finishing main" << std::endl;
  thread1.cancel();
  OpenThreads::Thread::microSleep(1000*1);
}
