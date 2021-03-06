#pragma once

#include <sbtypes.h>

#include <map>

#pragma comment(lib, "winmm.lib")

namespace Scenebuilder{;

/// timer callback
class TimerCallback{
public:
	virtual void OnTimer() = 0;

};

class Timer{
public:
	typedef map<uint, Timer*>  IdMap;
	static IdMap idMap;

	uint			id;			///< timer ID
	uint			res;		///< timer resolution
	TimerCallback*	callback;	///< timer callback function
	
public:
	void SetCallback  (TimerCallback* _callback);
	void SetResolution(uint res);
	
	/** @brief start the timer
		@param delay    timer period [ms]
	 */
	bool	Start(uint delay);

	/// stop the timer
	void	Stop();

	/// get system time in [ms]
	static uint	GetTime();

	 Timer();
	~Timer();
};

}
