#pragma once

#include <sbtypes.h>

#include <map>

namespace Scenebuilder{;

class EventGroup;

/**
	event object
 */
class Event{
public:
	string	     name;
	vector<EventGroup*>  groups;

public:
	bool	Create(bool manual = false);

	void	Close();

	/** イベントを待つ
		@param timeout  タイムアウト[ms]  0にするとウェイトせずに即座に返る
		@return	        取得に成功したらtrue, タイムアウトその他のエラーはfalse
	 */
	bool Wait(uint timeout = 1000);

	/** 複数のイベントを待つ  * caution: windows only
		@param  evBegin		イベントへのポインタを格納した配列の先頭
		@param  num			イベント数
		@param  wait_all	trueなら全てのイベントが発行するまで待つ．falseならいずれかのイベントが発行するまで待つ
		@return				タイムアウトしたら-1，wait_allがtrueで全てのイベントが発行したら0，wait_allがfalseなら発行したイベントのインデックス
	 */
	static int Wait(Event** evBegin, uint num, uint timeout, bool wait_all);

	/** イベントをセット
	 */
	void Set();

	/** セットされているかどうか
		- timeoutを0としてWaitするのと等価
	 */
	bool IsSet();

	/** イベントをリセット
		- manual resetの場合に限る
	 */
	void Reset();

	Event();
	~Event();
};

/**
	group of event objects
 */
class EventGroup : public Event, public vector<Event*>{
public:
	void Add(Event* ev);

	int Wait(uint timeout);

};

}
