#pragma once

#include <sbtypes.h>

/** TCPクライアント **/

namespace Scenebuilder{;

class TCPClientImpl;

class TCPReceiveCallback{
public:
	virtual void OnReceive(const byte* buf, size_t len) = 0;
};

class TCPClient{
public:
	TCPClientImpl* impl;	

public:
	bool Connect    (const char* host, int port);
	void Disconnect ();
	void Send       (const byte* buf, size_t len);
	void SetCallback(TCPReceiveCallback* cb);

	template<class T>
	void Send(const T& val){
		Send((const byte*)&val, sizeof(T));
	}

	 TCPClient();
	~TCPClient();
};

}

