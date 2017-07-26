#pragma once

#include <sbtypes.h>

/** TCP **/

namespace Scenebuilder{;

class TCPClientImpl;
class TCPServerImpl;
class TCPSessionImpl;

class TCPServerCallback{
public:
	/* 受信コールバック
	   rxBuf  受信バッファ
	   rxLen  受信バイト数
	   txBuf  送信バッファ
	   txLen  送信バイト数

	   送信バイト数が0なら再度受信，0以外なら送信開始
	 */
	virtual void OnRead(const byte* rxBuf, size_t rxLen, byte* txBuf, size_t& txLen) = 0;

};

class TCPClientCallback{
public:
	virtual void OnRead(const byte* buf, size_t len) = 0;
};



class TCPServer{
public:
	TCPServerImpl* impl;

public:
	/// start running
	void Run(int port);

	/// stop
	void Stop();

	void SetCallback(TCPServerCallback* cb);

	 TCPServer();
	~TCPServer();
};

class TCPClient{
public:
	TCPClientImpl* impl;	

public:
	bool Connect    (const char* host, int port);
	void Disconnect ();
	void Send       (const byte* buf, size_t len);
	void SetCallback(TCPClientCallback* cb);

	template<class T>
	void Send(const T& val){
		Send((const byte*)&val, sizeof(T));
	}

	 TCPClient();
	~TCPClient();
};

}

