#include <sbudp.h>

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

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class UDPSenderImpl{
public:
	io_service         ioService;
	system::error_code err;
	udp::socket        sock;
	udp::endpoint      targetEp;
	bool               broadcast;
	string             host;
	int                port;

public:
	void Connect(const char* _host, int _port, bool _broadcast){
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

	void Disconnect(){
		sock.close(err);
	}

	void Send(const byte* data, size_t len){
		sock.send_to(boost::asio::buffer(data, len), targetEp);
	}

	UDPSenderImpl() : sock(ioService){
	}

};

///////////////////////////////////////////////////////////////////////////////////////////////////

UDPSender::UDPSender(){
	impl = new UDPSenderImpl();
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
	size_t              receivedLen;
	io_service          ioService;
	udp::socket*        sock;
	system::error_code  err;
	udp::endpoint       remoteEp;
	boost::thread       threadIoService;
	int                 port;
	byte                buf[1024];
	
	UDPReceiveCallback* callback;

	void OnReceive(const system::error_code& err, size_t len){
		if(err == boost::asio::error::operation_aborted){
			std::cout << "Receive callback aborted." << std::endl;
			return;
		}
		
		if(callback)
			callback->OnReceive(buf, len);

		sock->async_receive_from(boost::asio::buffer(buf), remoteEp, boost::bind(&UDPReceiverImpl::OnReceive, this, _1, _2));
	}

public:
	void Connect(int _port){
		port = _port;

		sock = new udp::socket(ioService, udp::endpoint(udp::v4(), port));
		sock->async_receive_from(boost::asio::buffer(buf), remoteEp, boost::bind(&UDPReceiverImpl::OnReceive, this, _1, _2));
		threadIoService = thread(boost::bind(&io_service::run, &ioService));
	}

	void Disconnect(){
		sock->cancel();
		threadIoService.join();
		delete sock;
		sock = 0;
	}

	void SetCallback(UDPReceiveCallback* cb){
		callback = cb;
	}

	UDPReceiverImpl(){
		callback = 0;
	}

	~UDPReceiverImpl(){
	}

};

///////////////////////////////////////////////////////////////////////////////////////////////////

UDPReceiver::UDPReceiver(){
	impl = new UDPReceiverImpl();
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
