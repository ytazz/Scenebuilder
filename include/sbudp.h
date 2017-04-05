#pragma once

#include <sbtypes.h>

/** UDPクライアント **/

namespace Scenebuilder{;

class UDPSenderImpl;
class UDPReceiverImpl;

class UDPSender{
public:
	UDPSenderImpl* impl;	

public:
	void Connect   (const char* host, int port, bool broadcast);
	void Disconnect();
	void Send      (const byte* buf, size_t len);

	template<class T>
	void Send(const T& val){
		Send((const byte*)&val, sizeof(T));
	}

	 UDPSender();
	~UDPSender();
};

class UDPReceiveCallback{
public:
	virtual void OnReceive(const byte* buf, size_t len) = 0;
};

class UDPReceiver{
public:
	UDPReceiverImpl*    impl;	

public:
	void Connect    (int port);
	void Disconnect ();
	void SetCallback(UDPReceiveCallback* cb);

	 UDPReceiver();
	~UDPReceiver();
};

}

