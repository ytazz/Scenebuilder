#include <sbmessage.h>
#include <sbtimer.h>
#include <windows.h>

namespace Scenebuilder{;

Timer::IdMap Timer::idMap;

void CALLBACK TimerProc(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2){
	Timer::IdMap::iterator it = Timer::idMap.find(uTimerID);
	if(it != Timer::idMap.end()){
		Timer* timer = it->second;
		if(timer->callback)
			timer->callback->OnTimer();
	}
}

Timer::Timer(){
	callback = 0;
	id       = 0;
	res      = 10;
}

Timer::~Timer(){
	timeEndPeriod(res);
}

void Timer::SetCallback(TimerCallback* _callback){
	callback = _callback;
}

void Timer::SetResolution(uint _res){
	res = _res;
}

bool Timer::Start(uint delay){
	id = (int)timeSetEvent(delay, res, (LPTIMECALLBACK)TimerProc, NULL, TIME_PERIODIC);
	Timer::idMap[id] = this;
	return (id != 0);
}

void Timer::Stop(){
	timeKillEvent(id);
}

uint Timer::GetTime(){
	return timeGetTime();
}

}
