#include <sbudp.h>
#include <sbmessage.h>
#include <sbthread.h>
#include <sbevent.h>

#ifdef _WIN32
# ifndef _WIN32_WINNT
#  define _WIN32_WINNT 0x0600
# endif
#endif

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>
using namespace boost;
using namespace boost::asio;
using boost::asio::ip::udp;

#include <winsock2.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class UDPSenderImpl{
public:
	bool               broadcast;
	string             host;
	int                port;

public:
	virtual void Connect(const char* _host, int _port, bool _broadcast) = 0;
	virtual void Disconnect() = 0;
	virtual void Send(const byte* data, size_t len) = 0;
};

class UDPSenderImplAsio : public UDPSenderImpl{
public:
	io_service         ioService;
	system::error_code err;
	udp::socket        sock;
	udp::endpoint      targetEp;

public:
	virtual void Connect(const char* _host, int _port, bool _broadcast){
		port      = _port;
		host      = _host;
		broadcast = _broadcast;

		sock.open(udp::v4(), err);

		if(!err){
			if(broadcast){
				sock.set_option (udp::socket::reuse_address(true));
				sock.set_option (socket_base::broadcast(true));
				targetEp.address(ip::address_v4::broadcast());
			}
			else{
				targetEp.address(ip::address::from_string(host));
			}
			targetEp.port(port);
		}
	}

	virtual void Disconnect(){
		sock.close(err);
	}

	virtual void Send(const byte* data, size_t len){
		sock.send_to(boost::asio::buffer(data, len), targetEp);
	}

	UDPSenderImplAsio() : sock(ioService){
	}
};

class UDPSenderImplWinsock : public UDPSenderImpl{
public:
	WSADATA      wsa;
	sockaddr_in  siRemote;
	SOCKET       sock;

public:
	virtual void Connect(const char* _host, int _port, bool _broadcast){
		port      = _port;
		host      = _host;
		broadcast = _broadcast;

		if(WSAStartup(MAKEWORD(2,2), &wsa) != 0){
			Message::Error("winsock initialization failed");
			return;
		}
		
		if((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR){
			Message::Error("socket creation failed");
			return;
		}

		memset((byte*)&siRemote, 0, sizeof(siRemote));
		siRemote.sin_family = AF_INET;
		siRemote.sin_port   = htons(port);
		siRemote.sin_addr.S_un.S_addr = inet_addr(host.c_str());
	}

	virtual void Disconnect(){
		closesocket(sock);
		WSACleanup();
	}

	virtual void Send(const byte* data, size_t len){
		sendto(sock, (const char*)data, len, 0, (sockaddr*)&siRemote, sizeof(siRemote));
	}

	UDPSenderImplWinsock(){
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

UDPSender::UDPSender(bool use_asio){
	if(use_asio)
		 impl = new UDPSenderImplAsio   ();
	else impl = new UDPSenderImplWinsock();
}

UDPSender::~UDPSender(){
	delete impl;
}

void UDPSender::Connect(const char* host, int port, bool broadcast){
	impl->Connect(host, port, broadcast);
}

void UDPSender::Disconnect(){
	impl->Disconnect();
}

void UDPSender::Send(const byte* buf, size_t len){
	impl->Send(buf, len);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

class UDPReceiverImpl{
public:
	int                 port;
	
	UDPReceiveCallback* callback;

public:
	virtual void Connect(int _port) = 0;
	virtual void Disconnect() = 0;

	void SetCallback(UDPReceiveCallback* cb){
		callback = cb;
	}

	UDPReceiverImpl(){
		callback = 0;
	}
	~UDPReceiverImpl(){
	}

};

class UDPReceiverImplAsio : public UDPReceiverImpl{
public:
	size_t              receivedLen;
	io_service          ioService;
	udp::socket*        sock;
	system::error_code  err;
	udp::endpoint       remoteEp;
	boost::thread       threadIoService;
	byte                buf[1024*1024];

public:
	void OnReceive(const system::error_code& err, size_t len){
		if(err == boost::asio::error::operation_aborted){
			std::cout << "Receive callback aborted." << std::endl;
			return;
		}
		
		if(callback)
			callback->OnReceive(buf, len);

		sock->async_receive_from(boost::asio::buffer(buf), remoteEp, boost::bind(&UDPReceiverImplAsio::OnReceive, this, _1, _2));
	}

public:
	virtual void Connect(int _port){
		port = _port;

		sock = new udp::socket(ioService, udp::endpoint(udp::v4(), port));
		sock->async_receive_from(boost::asio::buffer(buf), remoteEp, boost::bind(&UDPReceiverImplAsio::OnReceive, this, _1, _2));
		threadIoService = thread(boost::bind(&io_service::run, &ioService));
	}

	virtual void Disconnect(){
		sock->cancel();
		threadIoService.join();
		delete sock;
		sock = 0;
	}

	UDPReceiverImplAsio(){
	}
	~UDPReceiverImplAsio(){
	}

};

class UDPReceiverImplWinsock : public UDPReceiverImpl, public Thread{
public:
	WSADATA       wsa;
	sockaddr_in   siRemote;
	SOCKET        sock;
	vector<byte>  buffer;
	Event         evDisconnect;

public:

public:
	virtual void Func(){
		while(!evDisconnect.IsSet()){
			int len = recv(sock, (char*)&buffer[0], buffer.size(), 0);
			if(len > 0)
				callback->OnReceive(&buffer[0], len);
		}
	}

	virtual void Connect(int _port){
		port      = _port;

		if(WSAStartup(MAKEWORD(2,2), &wsa) != 0){
			Message::Error("winsock initialization failed");
			return;
		}
		
		if((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR){
			Message::Error("socket creation failed");
			return;
		}

		memset((byte*)&siRemote, 0, sizeof(siRemote));
		siRemote.sin_family = AF_INET;
		siRemote.sin_port   = htons(port);
		siRemote.sin_addr.S_un.S_addr = INADDR_ANY;

		::bind(sock, (sockaddr*)&siRemote, sizeof(siRemote));

		buffer.resize(1024*1024);
		Run();
	}

	virtual void Disconnect(){
		evDisconnect.Set();
		//Join();

		closesocket(sock);
		WSACleanup();
	}

	UDPReceiverImplWinsock(){
	}
	~UDPReceiverImplWinsock(){
	}

};

///////////////////////////////////////////////////////////////////////////////////////////////////

UDPReceiver::UDPReceiver(bool use_asio){
	if(use_asio)
		 impl = new UDPReceiverImplAsio   ();
	else impl = new UDPReceiverImplWinsock();
}

UDPReceiver::~UDPReceiver(){
	delete impl;
}

void UDPReceiver::Connect(int port){
	impl->Connect(port);
}

void UDPReceiver::Disconnect(){
	impl->Disconnect();
}

void UDPReceiver::SetCallback(UDPReceiveCallback* cb){
	impl->SetCallback(cb);
}

}
