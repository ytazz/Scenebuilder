﻿#include <sbthread.h>
#include <sbmessage.h>

// want to migrate to std, but join with timeout is supported by boost::thread only
#include <boost/thread.hpp>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class ThreadImpl{
	boost::thread	th;
public:
	void Run(Thread* func){
		th = boost::thread(boost::ref(*func));
	}
	bool Join(int timeout){
		if(timeout == -1){
		     th.join();
			 return true;
		}
		else{
			return th.try_join_for(boost::chrono::milliseconds(timeout));
		}
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

Thread::Thread(){
	impl = new ThreadImpl();
}

Thread::~Thread(){
	delete impl;
}

void Thread::operator()(){
	Func();
}

void Thread::Run(){
	impl->Run(this);
}

bool Thread::Join(int timeout){
	return impl->Join(timeout);
}

void Thread::SleepUS(int us){
	boost::this_thread::sleep_for(boost::chrono::microseconds(us));
}

}
