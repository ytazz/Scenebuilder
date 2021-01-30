#include <sbmessage.h>
#include <sbtimer.h>

#ifdef _WIN32
# include <windows.h>
#else
# include <sys/time.h>
#endif

namespace Scenebuilder{;

Timer::IdMap Timer::idMap;

#ifdef _WIN32
void CALLBACK TimerProc(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2){
	Timer::IdMap::iterator it = Timer::idMap.find(uTimerID);
	if(it != Timer::idMap.end()){
		Timer* timer = it->second;
		if(timer->callback)
			timer->callback->OnTimer();
	}
}
#endif

Timer::Timer(){
	callback = 0;
	id       = 0;
	res      = 10;
}

Timer::~Timer(){
#ifdef _WIN32
	timeEndPeriod(res);
#endif
}

void Timer::SetCallback(TimerCallback* _callback){
	callback = _callback;
}

void Timer::SetResolution(uint _res){
	res = _res;
}

bool Timer::Start(uint delay){
#ifdef _WIN32
	id = (int)timeSetEvent(delay, res, (LPTIMECALLBACK)TimerProc, NULL, TIME_PERIODIC);
#endif
	Timer::idMap[id] = this;
	return (id != 0);
}

void Timer::Stop(){
#ifdef _WIN32
	timeKillEvent(id);
#endif
}

uint Timer::GetTime(){
#ifdef _WIN32
	return timeGetTime();
#else
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ts.tv_nsec/1000000;
#endif
	//return 0;
}

int Timer::CountUS(){
	auto t = std::chrono::high_resolution_clock::now();
	auto duration = t - last;
	int us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

	last = t;

	return us;
}

}
