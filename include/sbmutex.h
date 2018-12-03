#pragma once

#include <sbtypes.h>

#include <map>

namespace Scenebuilder{;

/**
	Windows Mutexのラッパー
	- 名前なしMutexは同一プロセス内のスレッド同期に使用
	- 名前ありMutexはプロセス間同期に使用
	- 共有メモリ上にMutexオブジェクトを置くことを想定すると，
	　ハンドルはプロセス固有のため共有メモリに置けない（i.e., メンバ変数にできない）ので
	  staticメンバ（プロセス固有メモリ空間に置かれる）にハンドルデータベースを保持する
 */
class Mutex{
private:
	Mutex(const Mutex& ){}	///< 誤って呼ばれないように

protected:
	Mutex*  target;

	bool	Create();
	void	Close();

public:
	str256_t	name;					///< ミューテックスの名前

	/** ロックする
		@param timeout	タイムアウト[ms]
		@return		取得に成功したらtrue, タイムアウトその他のエラーはfalse
	 */
	bool Lock(uint timeout = 1000);

	/** アンロックする
	 */
	void Unlock();
	
	Mutex();

	// コンストラクタでLock，デストラクタでUnlockする
	 Mutex(Mutex* _target);
	~Mutex();
};

}
