#ifndef SB_THREAD_H
#define SB_THREAD_H

#include <sbscene.h>

namespace Scenebuilder{;

class ThreadImpl;

/** Thread
	- boost::threadのラッパ
	- 
 **/
class Thread{
protected:
	ThreadImpl* impl;

public:
	void Run ();
	bool Join(int timeout = -1);

	void operator()();
	
	virtual void Func() = 0;
	
	Thread();
	virtual ~Thread();
};

}

#endif
