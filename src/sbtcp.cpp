#include <sbtcp.h>
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

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class TCPSessionImpl : public UTRefCount{
public:
	TCPServerImpl*  server;
	int				id;
	io_service&		ioService;
	tcp::socket		sock;

	byte            rxBuf[1024];
	byte            txBuf[1024];
	size_t          rxLen;
	size_t          txLen;
	size_t          txPos;

public:
	void OnRead (const system::error_code& err, size_t len);
	void OnWrite(const system::error_code& err, size_t len);

public:
	int			 GetID    ();
	tcp::socket& GetSocket();
	void         Start    ();
	void         Stop     ();

	 TCPSessionImpl(TCPServerImpl* _server, int _id, io_service& ios);
	~TCPSessionImpl();
};

typedef vector< UTRef<TCPSessionImpl> > TCPSessions;

///////////////////////////////////////////////////////////////////////////////////////////////////

class TCPServerImpl{
public:
	TCPServer*		    owner;
	TCPServerCallback*  callback;

	/// boost::asio objects
	io_service		ioService;
	tcp::acceptor*	acceptor;

	/// thread to run io_service
	thread			threadIoService;

	/// array of sessions
	TCPSessions		sessions;

	bool			running;

	/** internal callbacks **/
	void OnAccept(const system::error_code& error);
	void OnStop  ();

public:
	void StartSession();
	void Run         (int port);
	void Stop        ();
    void SetCallback (TCPServerCallback* cb);

	 TCPServerImpl();
	~TCPServerImpl();
};

///////////////////////////////////////////////////////////////////////////////////////////////////

class TCPClientImpl{
public:
	io_service         ioService;
	io_service::work*  work;
	system::error_code err;
	tcp::socket*       sock;
	deadline_timer*	   timer;
	string             host;
	int                port;

	volatile bool      connecting;
	volatile bool      connected;
	size_t             receivedLen;
	boost::thread      threadIoService;
	byte               buf[1024];
	
	TCPClientCallback* callback;

	void OnConnect       (const system::error_code& err);
	void OnConnectTimeout(const system::error_code& err);
	void OnReceive       (const system::error_code& err, size_t len);

public:
	void SetCallback(TCPClientCallback* cb);
	bool Connect    (const char* _host, int _port);
	void Disconnect ();
	void Send       (const byte* data, size_t len);

	TCPClientImpl();

};

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPSessionImpl::TCPSessionImpl(TCPServerImpl* _server, int _id, io_service& ios):server(_server), id(_id), ioService(ios), sock(ios){

}

TCPSessionImpl::~TCPSessionImpl(){

}

void TCPSessionImpl::OnRead(const system::error_code& err, size_t len){
	if(!err){
		Message::Extra("session %d: %d byte received", GetID(), len);

		rxLen = len;
		txLen = 0;

		if(server->callback)
			server->callback->OnRead(rxBuf, rxLen, txBuf, txLen);

		if(txLen == 0){
			sock.async_read_some(boost::asio::buffer(rxBuf), boost::bind(&TCPSessionImpl::OnRead, this, _1, _2));
		}
		else{
			txPos = 0;
			sock.async_write_some(boost::asio::buffer(txBuf), boost::bind(&TCPSessionImpl::OnWrite, this, _1, _2));
		}
	}
	else{
		Message::Error("session %d: error in read (%s)", GetID(), err.message().c_str());
		Stop();
	}
}

void TCPSessionImpl::OnWrite(const system::error_code& err, size_t len){
	if (!err){
		Message::Extra("session %d: %d byte sent", GetID(), len);

		txPos += len;

		if(txPos < txLen){
			// 送信コンテンツがあれば送信開始
			sock.async_write_some(boost::asio::buffer(&txBuf[txPos], txLen - txPos), boost::bind(&TCPSessionImpl::OnWrite, this, _1, _2));
		}
		else{
			// 次のリクエストヘッダを受け付け
			sock.async_read_some(boost::asio::buffer(rxBuf), boost::bind(&TCPSessionImpl::OnRead, this, _1, _2));
		}
	}
	else{
		Message::Error("session %d: error in write (%s)", GetID(), err.message().c_str());
		Stop();
	}
}
	
int TCPSessionImpl::GetID(){ return id; }

tcp::socket& TCPSessionImpl::GetSocket(){ return sock; }
	
void TCPSessionImpl::Start(){
	Message::Out("session %d: started", GetID());

	// ヘッダ受信開始
	sock.async_read_some(boost::asio::buffer(rxBuf), boost::bind(&TCPSessionImpl::OnRead, this, _1, _2));
}

void TCPSessionImpl::Stop(){
	if(sock.is_open())
		sock.close();
	Message::Out("session %d: stopped", GetID());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPServerImpl::TCPServerImpl(){
	acceptor = 0;
	running  = false;
	callback = 0;
}

TCPServerImpl::~TCPServerImpl(){
	if(acceptor)
		delete acceptor;
}

void TCPServerImpl::OnAccept(const system::error_code& error){
	if (!error){
		sessions.back()->Start();

		// wait for another connection request
		StartSession();
	}
}

void TCPServerImpl::OnStop(){
	// accept終了
	acceptor->close();

	// 走っているセッションを終了
	for(TCPSessions::iterator it = sessions.begin(); it != sessions.end(); it++)
		(*it)->Stop();
}

void TCPServerImpl::StartSession(){
	TCPSessionImpl* s = new TCPSessionImpl(this, (int)sessions.size(), ioService);
	sessions.push_back(s);
	acceptor->async_accept(sessions.back()->GetSocket(), boost::bind(&TCPServerImpl::OnAccept, this, _1));
}

void TCPServerImpl::Run(int port){
	if(running)
		return;
	
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

void TCPServerImpl::Stop(){
	if(!running)
		return;

	// 稼働中のセッションに停止要求
	ioService.post(boost::bind(&TCPServerImpl::OnStop, this));
		
	// サービスの停止を待つ
	threadIoService.join();
		
	// セッションクリア
	sessions.clear();
		
	Message::Out("server: stopped");
	running = false;
}

void TCPServerImpl::SetCallback(TCPServerCallback* cb){
	callback = cb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPClientImpl::TCPClientImpl(){
	timer    = 0;
	sock     = 0;
	callback = 0;
}

void TCPClientImpl::OnConnect(const system::error_code& err){
	timer->cancel();
	if(!err)
		connected = true;
	connecting = false;
}

void TCPClientImpl::OnConnectTimeout(const system::error_code& err){
	if(!err){
		sock->close();
		connecting = false;
		connected  = false;
	}
}

void TCPClientImpl::OnReceive(const system::error_code& err, size_t len){
	if(err == boost::asio::error::operation_aborted){
		return;
	}
		
	if(callback)
		callback->OnRead(buf, len);

	sock->async_receive(boost::asio::buffer(buf), boost::bind(&TCPClientImpl::OnReceive, this, _1, _2));
}

void TCPClientImpl::SetCallback(TCPClientCallback* cb){
	callback = cb;
}

bool TCPClientImpl::Connect(const char* _host, int _port){
	if(sock)
		Disconnect();

	host  = _host;
	port  = _port;

	sock  = new tcp::socket     (ioService);
	timer = new deadline_timer  (ioService);
	work  = new io_service::work(ioService);
	connecting = true;
	connected  = false;
	sock->async_connect(tcp::endpoint(ip::address::from_string(host), port), boost::bind(&TCPClientImpl::OnConnect, this, _1));
	timer->expires_from_now(boost::posix_time::seconds(1));
	timer->async_wait(boost::bind(&TCPClientImpl::OnConnectTimeout, this, _1));

	threadIoService = thread(boost::bind(&io_service::run, &ioService));

	while(connecting);
	if(!connected){
		delete work;
		threadIoService.join();
		delete sock;
		sock = 0;
		return false;
	}
		
	sock->async_receive(boost::asio::buffer(buf), boost::bind(&TCPClientImpl::OnReceive, this, _1, _2));
		
	return true;
}

void TCPClientImpl::Disconnect(){
	if(!sock)
		return;
	sock->cancel();
	delete work;
	threadIoService.join();
	delete sock;
	sock = 0;
}

void TCPClientImpl::Send(const byte* data, size_t len){
	if(!sock)
		return;
	sock->send(boost::asio::buffer(data, len));
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPServer::TCPServer(){
	impl = new TCPServerImpl();
	impl->owner = this;
}

TCPServer::~TCPServer(){
	delete impl;
}

void TCPServer::Run(int port){
	impl->Run(port);
}

void TCPServer::Stop(){
	impl->Stop();
}

void TCPServer::SetCallback(TCPServerCallback* cb){
	impl->SetCallback(cb);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TCPClient::TCPClient(){
	impl = new TCPClientImpl();
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

void TCPClient::Send(const byte* buf, size_t len){
	impl->Send(buf, len);
}

void TCPClient::SetCallback(TCPClientCallback* cb){
	impl->SetCallback(cb);
}

}
