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
	virtual void OnTCPServerReceive(const byte* rxBuf, size_t  rxLen, byte* txBuf, size_t& txLen) = 0;

};

class TCPClientCallback{
public:
	virtual void OnTCPClientReceive(const byte* buf, size_t len) = 0;
};

class TCPServer{
public:
	TCPServerImpl* impl;

	int  listenInterval;    ///< polling interval for listen [ms] (for Winsock)
	int  receiveInterval;   ///< polling interval for recv   [ms] (for Winsock)

public:
	/// start running
	void Start(int port);

	/// stop
	void Stop();

	void SetCallback(TCPServerCallback* cb);

	 TCPServer(bool use_asio = true);
	~TCPServer();
};

class TCPClient{
public:
	TCPClientImpl* impl;

	int  connectTimeout;
	int  receiveInterval;
	int  sendTimeout;

public:
	bool Connect    (const char* host, int port);
	void Disconnect ();
	bool Send       (const byte* buf, size_t len);
	void SetCallback(TCPClientCallback* cb);

	template<class T>
	void Send(const T& val){
		Send((const byte*)&val, sizeof(T));
	}

	 TCPClient(bool use_asio = true);
	~TCPClient();
};

}

