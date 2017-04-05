#pragma once

#include <sbtypes.h>

#include <map>

#pragma comment(lib, "winmm.lib")

namespace Scenebuilder{;

/// タイマコールバック
class TimerCallback{
public:
	virtual void OnTimer() = 0;

};

class Timer{
public:
	typedef map<uint, Timer*>  IdMap;
	static IdMap idMap;

	uint			id;			///< タイマID
	uint			res;		///< タイマ分解能
	TimerCallback*	callback;	///< タイマコールバック関数
	
public:
	void SetCallback  (TimerCallback* _callback);
	void SetResolution(uint res);
	
	/** @brief タイマ始動
		@param delay	タイマ周期[ms]
	 */
	bool	Start(uint delay);

	/// タイマ停止
	void	Stop();

	/// システム時刻[ms]
	static uint	GetTime();

	 Timer();
	~Timer();
};

}
