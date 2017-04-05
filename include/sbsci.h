#pragma once

#include <sbtypes.h>
#include <sbevent.h>

namespace Scenebuilder{;

class Sci{
public:
	void*	handle;       ///< ハンドル
	void*   overlapped;
	bool	isOnline;     ///< 接続中
	
	bool         txBufferEnabled;
	vector<byte> txBuffer;

	uint    rxTotal;    ///< 総受信バイト数
	uint    txTotal;    ///< 総送信バイト数
	
public:
	
	/**	@brief 初期化
		@param	name	オープンするデバイスファイル．デフォルトはCOM1
		@param	baud	ボーレート [bps]
	 */
	void Init(const string& port, int baud = 57600, int byteSize = 8, int stopBits = 1);

	/** @brief 終了
		接続を切断する．デストラクタからも呼ばれる．
	 */
	void Terminate();

	/** @接続中かどうか
	 **/
	bool IsOnline(){ return isOnline; }

	/** 出力バッファを有効/無効化
	     バッファ有効時はOut呼び出しで出力バッファに格納し，FlushTxBuffer呼び出しで実際の送信を行う
     */
	void   EnableTxBuffer(bool on = true);

	/// 出力バッファをフラッシュ
	size_t FlushTxBuffer ();

	/** @brief 送信
		@param c	送信データ
		@param n	送信要求バイト数
		@return		実際の送信バイト数
	 */
	size_t Out(const byte* c, size_t n);

	/** @brief 送信(template版)
		@param c	送信データ
		T型のデータを1つ送信する．(double x = 1.0; Out(x);)
	 */
	template<class T>
	size_t Out(const T& c){
		return Out((const byte*)&c, sizeof(T));
	}

	/** @brief 送信(文字列版)
	 */
	size_t TextOut(const char* c){
		return Out((const byte*)c, strlen(c));
	}
	
	/** @brief 受信
		@param c	受信バッファ
		@param n	受信要求バイト数
		@param full 要求バイト数受信するまで戻らない
		@return		実際の受信バイト数
	 */
	size_t In(byte* c, size_t n, bool full = true);

	/**	@brief 受信(template版)
		@param c	受信変数
		T型のデータを1つ受信する．(double x; In(&x);)
	 */
	template<class T>
	size_t In(T* c, bool full = true){
		return In((byte*)c, sizeof(T), true);
	}

	/** @brief 受信/送信バッファにたまっているデータのバイト数を取得する
	 */
	void CountBytes(size_t* rx, size_t* tx);
	
	 Sci();
	~Sci();
};

}
