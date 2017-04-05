#include <sbtcp.h>

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
	
	TCPReceiveCallback* callback;

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
		
		if(callback)
			callback->OnReceive(buf, len);

		sock->async_receive(boost::asio::buffer(buf), boost::bind(&TCPClientImpl::OnReceive, this, _1, _2));
	}

public:
	void SetCallback(TCPReceiveCallback* cb){
		callback = cb;
	}

	bool Connect(const char* _host, int _port){
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

	void Disconnect(){
		if(!sock)
			return;
		sock->cancel();
		delete work;
		threadIoService.join();
		delete sock;
		sock = 0;
	}

	void Send(const byte* data, size_t len){
		if(!sock)
			return;
		sock->send(boost::asio::buffer(data, len));
	}

	TCPClientImpl(){
		timer    = 0;
		sock     = 0;
		callback = 0;
	}

};

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

void TCPClient::SetCallback(TCPReceiveCallback* cb){
	impl->SetCallback(cb);
}

}
