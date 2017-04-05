#pragma once

#include <sbtypes.h>

#include <map>

namespace Scenebuilder{;

/**
	Windows Eventのラッパー
 */
class Event{
protected:
	typedef std::map<void*, void*>	HandleDB;
	static HandleDB	handles;

public:
	string	name;

public:
	bool	Create(bool manual = false);

	void	Close();

	/** イベントを待つ
		@param timeout  タイムアウト[ms]  0にするとウェイトせずに即座に返る
		@return	        取得に成功したらtrue, タイムアウトその他のエラーはfalse
	 */
	bool Wait(uint timeout = 1000);

	/** 複数のイベントを待つ
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

	/* 内部ハンドルを取得
	 */
	void*	GetHandle();

	Event();
	~Event();
};

}
