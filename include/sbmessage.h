#pragma once

/**
	コンソールやファイルへのメッセージ出力機能
 */

#include <sbtypes.h>
#include <sbcriticalsection.h>

#include <stdio.h>
#include <stdarg.h>

namespace Scenebuilder{;

/** メッセージ出力のためのクラス
	- すべてstatic関数
 **/

/// メッセージコールバック
class MessageCallback{
public:
	virtual void OnMessage(int lv, const char* str) = 0;
};

class Message{
protected:
	static int				verboseLevel;
	static ostream*			os;
	static MessageCallback*	callback;
	static CriticalSection	cs;

public:
	struct Level{
		enum{
			Error  = 0,
			Normal = 1,
			Extra  = 2,
		};
	};

	/** 出力先ストリームの設定

	 **/
	static void SetStream(ostream* _os);

	/** コールバック関数を設定
	 */
	static void SetCallback(MessageCallback* cb);

	/** 饒舌レベルの設定
		verbose level >= message level
		の場合のみメッセージが出力される
	 **/
	static void SetVerboseLevel(int level);
	static int	GetVerboseLevel();

	/** メッセージを出力
	 **/
	static void Out  (const char*    str, ...);
	static void Error(const char*    str, ...);
	static void Extra(const char*    str, ...);
};

}
