#include <sbtcp.h>
#include <sbthread.h>
#include <sbevent.h>
#include <sbmessage.h>

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
using boost::asio::ip::tcp;

#include <winsock2.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class TCPSessionImpl : public UTRefCount{
public:
	TCPServerImpl*  server;
	int				id;

	byte            rxBuf[1024*1024];
	byte            txBuf[1024*1024];
	size_t          rxLen;
	size_t          txLen;
	size_t          txPos;

public:
	int	GetID(){ return id; }

	virtual void Start() = 0;
	virtual void Stop () = 0;

	TCPSessionImpl(TCPServerImpl* _server, int _id):server(_server), id(_id){}
	virtual ~TCPSessionImpl(){}
};

typedef vector< UTRef<TCPSessionImpl> > TCPSessions;

class TCPServerImpl{
public:
	TCPServer*		    owner;
	TCPServerCallback*  callback;
	int                 port;

	/// array of sessions
	TCPSessions		sessions;

	bool			running;

public:
	void SetCallback (TCPServerCallback* cb){ callback = cb; }

	virtual void Start(int port) = 0;
	virtual void Stop ()         = 0;
	
	TCPServerImpl(){
		running  = false;
		callback = 0;
	}
	virtual ~TCPServerImpl(){}
};

class TCPClientImpl{
public:
	TCPClient*         owner;
	string             host;
	int                port;

	byte               rxBuf[1024*1024];
	size_t             rxLen;
	
	volatile bool      connecting;
	volatile bool      connected;
	
	TCPClientCallback* callback;

public:
	void SetCallback(TCPClientCallback* cb){
		callback = cb;
	}

	virtual bool Connect    (const char* _host, int _port) = 0;
	virtual void Disconnect () = 0;
	virtual bool Send       (const byte* data, size_t len) = 0;

	TCPClientImpl(){
		callback = 0;
	}
	virtual ~TCPClientImpl(){}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

class TCPSessionImplAsio : public TCPSessionImpl{
public:
	io_service&		ioService;
	tcp::socket		sock;

public:
	tcp::socket& GetSocket(){ return sock; }

	void OnRead (const system::error_code& err, size_t len){
		if(!err){
			Message::Extra("session %d: %d byte received", GetID(), len);

			rxLen = len;
			txLen = 0;

			if(server->callback)
				server->callback->OnTCPServerReceive(rxBuf, rxLen, txBuf, txLen);

			if(txLen == 0){
				sock.async_read_some(boost::asio::buffer(rxBuf), boost::bind(&TCPSessionImplAsio::OnRead, this, _1, _2));
			}
			else{
				txPos = 0;
				sock.async_write_some(boost::asio::buffer(txBuf), boost::bind(&TCPSessionImplAsio::OnWrite, this, _1, _2));
			}
		}
		else{
			Message::Error("session %d: error in read (%s)", GetID(), err.message().c_str());
			Stop();
		}
	}
	void OnWrite(const system::error_code& err, size_t len){
		if (!err){
			Message::Extra("session %d: %d byte sent", GetID(), len);

			txPos += len;

			if(txPos < txLen){
				// 送信コンテンツがあれば送信開始
				sock.async_write_some(boost::asio::buffer(&txBuf[txPos], txLen - txPos), boost::bind(&TCPSessionImplAsio::OnWrite, this, _1, _2));
			}
			else{
				// 次のリクエストヘッダを受け付け
				sock.async_read_some(boost::asio::buffer(rxBuf), boost::bind(&TCPSessionImplAsio::OnRead, this, _1, _2));
			}
		}
		else{
			Message::Error("session %d: error in write (%s)", GetID(), err.message().c_str());
			Stop();
		}
	}

	virtual void Start(){
		Message::Out("session %d: started", GetID());

		// ヘッダ受信開始
		sock.async_read_some(boost::asio::buffer(rxBuf), boost::bind(&TCPSessionImplAsio::OnRead, this, _1, _2));
	}
	virtual void Stop(){
		if(sock.is_open())
			sock.close();
		Message::Out("session %d: stopped", GetID());
	}

	TCPSessionImplAsio(TCPServerImpl* _server, int _id, io_service& ios):TCPSessionImpl(_server, _id), ioService(ios), sock(ios){}
	virtual ~TCPSessionImplAsio(){}
};

class TCPServerImplAsio : public TCPServerImpl{
public:
	/// boost::asio objects
	io_service		ioService;
	tcp::acceptor*	acceptor;

	/// thread to run io_service
	thread			threadIoService;

public:
	/** internal callbacks **/
	void OnAccept(const system::error_code& error){
		if (!error)
			sessions.back()->Start();

		// wait for another connection request
		StartSession();
	}
	void OnStop  (){
		// accept終了
		acceptor->close();

		// 走っているセッションを終了
		for(TCPSessions::iterator it = sessions.begin(); it != sessions.end(); it++)
			(*it)->Stop();
	}

	void StartSession(){
		TCPSessionImplAsio* s = new TCPSessionImplAsio(this, (int)sessions.size(), ioService);
		sessions.push_back(s);
		acceptor->async_accept(s->GetSocket(), boost::bind(&TCPServerImplAsio::OnAccept, this, _1));
	}
	virtual void Start(int _port){
		if(running)
			return;

		port = _port;
	
		// create endpoint
		tcp::endpoint ep(tcp::v4(), port);

		// create acceptor
		acceptor = new tcp::acceptor(ioService, ep);

		// create new session
		StartSession();

		// バックグラウンドでio_service始動
		threadIoService = thread(boost::bind(&io_service::run, &ioService));

		Message::Out("server: listening on port %d", port);
		running = true;
	}
	virtual void Stop(){
		if(!running)
			return;

		// 稼働中のセッションに停止要求
		ioService.post(boost::bind(&TCPServerImplAsio::OnStop, this));
		
		// サービスの停止を待つ
		threadIoService.join();
		
		// セッションクリア
		sessions.clear();
		
		Message::Out("server: stopped");
		running = false;
	}

	TCPServerImplAsio(){
		acceptor = 0;
	}
	virtual ~TCPServerImplAsio(){
		if(acceptor)
			delete acceptor;
	}
};

class TCPClientImplAsio : public TCPClientImpl{
public:
	io_service         ioService;
	io_service::work*  work;
	system::error_code err;
	tcp::socket*       sock;
	deadline_timer*	   timer;

	boost::thread      threadIoService;

	void OnConnect(const system::error_code& err){
		timer->cancel();
		if(!err)
			connected = true;
		connecting = false;
	}
	void OnConnectTimeout(const system::error_code& err){
		if(!err){
			sock->close();
			connecting = false;
			connected  = false;
		}
	}
	void OnReceive(const system::error_code& err, size_t len){
		if(err == boost::asio::error::operation_aborted){
			return;
		}

		rxLen = len;
		
		if(callback)
			callback->OnTCPClientReceive(rxBuf, rxLen);

		sock->async_receive(boost::asio::buffer(rxBuf), boost::bind(&TCPClientImplAsio::OnReceive, this, _1, _2));
	}

public:
	virtual bool Connect(const char* _host, int _port){
		if(sock)
			Disconnect();

		host  = _host;
		port  = _port;

		sock  = new tcp::socket     (ioService);
		timer = new deadline_timer  (ioService);
		work  = new io_service::work(ioService);
		connecting = true;
		connected  = false;
		sock->async_connect(tcp::endpoint(ip::address::from_string(host), port), boost::bind(&TCPClientImplAsio::OnConnect, this, _1));
		timer->expires_from_now(boost::posix_time::seconds(1));
		timer->async_wait(boost::bind(&TCPClientImplAsio::OnConnectTimeout, this, _1));

		threadIoService = thread(boost::bind(&io_service::run, &ioService));

		while(connecting);
		if(!connected){
			delete work;
			threadIoService.join();
			delete sock;
			sock = 0;
			return false;
		}
		
		sock->async_receive(boost::asio::buffer(rxBuf), boost::bind(&TCPClientImplAsio::OnReceive, this, _1, _2));
		
		return true;

	}
	virtual void Disconnect(){
		if(!sock)
			return;
		sock->cancel();
		delete work;
		threadIoService.join();
		delete sock;
		sock = 0;
	}
	virtual bool Send(const byte* data, size_t len){
		if(!sock)
			return false;
		sock->send(boost::asio::buffer(data, len));
		return true;
	}

	TCPClientImplAsio(){
		timer    = 0;
		sock     = 0;
	}
	virtual ~TCPClientImplAsio(){}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

class TCPSessionImplWinsock : public TCPSessionImpl, public Thread{
public:
	SOCKET  sock;
	Event   evStop;

public:
	virtual void Func(){
		while(!evStop.IsSet()){			
			fd_set  fd;
			timeval to;
			while(!evStop.IsSet()){
				fd.fd_count = 1;
				fd.fd_array[0] = sock;
				to.tv_sec  = 0;
				to.tv_usec = 1000*server->owner->receiveInterval;
				int ret = select(0, &fd, 0, 0, &to);
				if(ret == 0){
					// timeout expired
				}
				else if(ret == SOCKET_ERROR){
					Message::Error("session %d: select failed", GetID());
				}
				else{
					ret = ::recv(sock, (char*)rxBuf, sizeof(rxBuf), 0);
					if(ret == SOCKET_ERROR){
						Message::Error("session %d: recv failed", GetID());
					}
					else{
						rxLen = ret;
						Message::Extra("session %d: %d byte received", GetID(), rxLen);

						if(rxLen > 0){
							if(server->callback)
								server->callback->OnTCPServerReceive(rxBuf, rxLen, txBuf, txLen);

							if(txLen > 0){
								int txSent = ::send(sock, (char*)txBuf, (int)txLen, 0);
								Message::Extra("session %d: %d bytes sent", GetID(), txSent);
							}
						}
					}
				}
			}
		}
	}

	virtual void Start(){
		Run();
	}
	virtual void Stop (){
		evStop.Set();
		Join();
		closesocket(sock);
		Message::Out("session %d: stopped", GetID());
	}

	TCPSessionImplWinsock(TCPServerImpl* _server, int _id):TCPSessionImpl(_server, _id){
		evStop.Create(true);
	}
	virtual ~TCPSessionImplWinsock(){}

};

class TCPServerImplWinsock : public TCPServerImpl, public Thread{
public:
	WSADATA      wsaData;
	sockaddr_in  si;
	SOCKET       sock;
	Event        evStop;

public:
	virtual void Func(){
		while(!evStop.IsSet()){
			if(::listen(sock, SOMAXCONN) == INVALID_SOCKET){
				Message::Error("listen failed");
				closesocket(sock);
				WSACleanup();
				break;
			}
		
			fd_set  fd;
			timeval to;
			while(!evStop.IsSet()){
				fd.fd_count = 1;
				fd.fd_array[0] = sock;
				to.tv_sec  = 0;
				to.tv_usec = 1000*owner->listenInterval;
				if(select(0, &fd, 0, 0, &to)){
					UTRef<TCPSessionImplWinsock> s = new TCPSessionImplWinsock(this, (int)sessions.size());
					if((s->sock = ::accept(sock, NULL, NULL)) == INVALID_SOCKET){
						Message::Error("accept failed");
						closesocket(sock);
						WSACleanup();
						break;
					}
					sessions.push_back(s);
					s->Start();
					break;
				}
			}
		}
	}

	virtual void Start(int _port){
		if(running)
			Stop();

		port = _port;

		if(WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
			Message::Error("WSAStartup failed");
			return;
		}

		if((sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR){
			Message::Error("socket creation failed");
			return;
		}

		si.sin_family      = AF_INET;
		si.sin_addr.s_addr = INADDR_ANY;
		si.sin_port        = htons(port);
		::bind(sock, (sockaddr*)&si, sizeof(si));

		running = true;
		Run();
	}

	virtual void Stop(){
		if(!running)
			return;

		for(uint i = 0; i < sessions.size(); i++)
			sessions[i]->Stop();

		evStop.Set();
		WSACleanup();
		Join();

		sessions.clear();
	}

	TCPServerImplWinsock(){
		evStop.Create(true);
	}
	virtual ~TCPServerImplWinsock(){}
};

class TCPClientImplWinsock : public TCPClientImpl, public Thread{
public:
	WSADATA      wsaData;
	sockaddr_in  si;
	SOCKET       sock;
	bool         running;
	Event        evStop;

public:
	virtual void Func(){
		fd_set  fd;
		timeval to;
		while(!evStop.IsSet()){
			fd.fd_count = 1;
			fd.fd_array[0] = sock;
			to.tv_sec  = 0;
			to.tv_usec = 1000*owner->receiveInterval;
			if(select(0, &fd, 0, 0, &to)){
				int ret = ::recv(sock, (char*)rxBuf, sizeof(rxBuf), 0);
				if(ret == SOCKET_ERROR){
					Message::Error("client: recv failed");
				}
				else{
					rxLen = ret;
					Message::Extra("client: %d byte received", rxLen);

					if(rxLen > 0){
						if(callback)
							callback->OnTCPClientReceive(rxBuf, rxLen);
					}
				}
			}
		}
	}

	virtual bool Connect(const char* _host, int _port){
		if(running)
			Disconnect();

		host = _host;
		port = _port;

		if(WSAStartup(MAKEWORD(2,2), &wsaData) != 0) {
			Message::Error("WSAStartup failed");
			return false;
		}

		if((sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == SOCKET_ERROR){
			Message::Error("socket creation failed");
			return false;
		}

		//unsigned long on = 1;
		//if(::ioctlsocket(sock, FIONBIO, &on) == SOCKET_ERROR){
		//	Message::Error("failed to make socket non-blocking");
		//	return false;
		//}

		si.sin_family      = AF_INET;
		si.sin_addr.s_addr = inet_addr(_host);
		si.sin_port        = htons(port);
		::connect(sock, (sockaddr*)&si, sizeof(si));

		//fd_set  fd;
		//timeval to;
		//fd.fd_count = 1;
		//fd.fd_array[0] = sock;
		//to.tv_sec  = 0;
		//to.tv_usec = 1000*owner->connectTimeout;
		//int ret = select(0, &fd, 0, 0, &to);
		//if(ret == 0){
		//	Message::Error("connect timeout");
		//	return false;
		//}
		//if(ret == SOCKET_ERROR){
		//	Message::Error("connect failed");
		//	return false;
		//}

		running = true;
		Run();

		return true;

	}
	virtual void Disconnect(){
		if(!running)
			return;

		evStop.Set();
		//Join();

		closesocket(sock);

		running = false;
	}
	virtual bool Send(const byte* data, size_t len){
		fd_set  fd;
		timeval to;
		
		fd.fd_count = 1;
		fd.fd_array[0] = sock;
		to.tv_sec  = 0;
		to.tv_usec = 1000*owner->sendTimeout;
		int ret = select(0, 0, &fd, 0, &to);
		if(ret == 0){
			// timeout expired
			Message::Error("client: select timed out");
			return false;
		}
		if(ret == SOCKET_ERROR){
			Message::Error("client: select failed");
			return false;
		}
		
		int txSent = ::send(sock, (char*)data, len, 0);
		Message::Extra("client: %d bytes sent", txSent);
		return true;
	}

	TCPClientImplWinsock(){
		running = false;
		evStop.Create(true);
	}
	virtual ~TCPClientImplWinsock(){}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPServer::TCPServer(bool use_asio){
	listenInterval  = 100;
	receiveInterval = 100;

	if(use_asio)
		 impl = new TCPServerImplAsio();
	else impl = new TCPServerImplWinsock();
	impl->owner = this;
}

TCPServer::~TCPServer(){
	delete impl;
}

void TCPServer::Start(int port){
	impl->Start(port);
}

void TCPServer::Stop(){
	impl->Stop();
}

void TCPServer::SetCallback(TCPServerCallback* cb){
	impl->SetCallback(cb);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPClient::TCPClient(bool use_asio){
	connectTimeout  = 1000;
	receiveInterval = 100;
	sendTimeout     = 100;

	if(use_asio)
		 impl = new TCPClientImplAsio();
	else impl = new TCPClientImplWinsock();
	impl->owner = this;
}

TCPClient::~TCPClient(){
	delete impl;
}

bool TCPClient::Connect(const char* host, int port){
	return impl->Connect(host, port);
}

void TCPClient::Disconnect(){
	impl->Disconnect();
}

bool TCPClient::Send(const byte* buf, size_t len){
	return impl->Send(buf, len);
}

void TCPClient::SetCallback(TCPClientCallback* cb){
	impl->SetCallback(cb);
}

}
