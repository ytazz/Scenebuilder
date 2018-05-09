#include <sbremote.h>
#include <sbmessage.h>

#ifdef _WIN32
# ifndef _WIN32_WINNT
#  define _WIN32_WINNT 0x0600
# endif
#endif

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace boost;
using namespace boost::asio;
using boost::asio::ip::tcp;

/** windows.h関係のundef **/
#undef GetObject
#undef CreateDialog

using namespace std;

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

class SessionBaseImpl{
public:
	SessionBase*	owner;
	int				id;
	io_service&		ioService;
	tcp::socket		sock;

public:
	void OnReadHeader   (const system::error_code& err){
		if(!err){
			Message::Extra("session %d: header received", GetID());

			// 受信コンテンツがない場合はそのままread contentsハンドラへ
			if(owner->rxHeader.sz == 0){
				OnReadContents(err);
			}
			else{
				owner->rxBuf.Reset(owner->rxHeader.sz);
				// コンテンツ受信開始
				async_read(sock, buffer(&owner->rxBuf[0], owner->rxBuf.size()), boost::bind(&SessionBaseImpl::OnReadContents, this, _1));
			}
		}
		else{
			Message::Error("session %d: error in read header (%s)", GetID(), err.message().c_str());
			Stop();
		}
	}
	void OnReadContents (const system::error_code& err){
		if(!err){
			Message::Extra("session %d: contents received", GetID());
			owner->txBuf.Reset();
			int err = owner->HandleRequest();

			// レスポンスヘッダ送信開始
			owner->txHeader.req = owner->rxHeader.req;
			owner->txHeader.err = err;
			owner->txHeader.sz  = (uint16_t)owner->txBuf.size();
			async_write(sock, buffer(&owner->txHeader, sizeof(ResponseHeader)), boost::bind(&SessionBaseImpl::OnWriteHeader, this, _1));
		}
		else{
			Message::Error("session %d: error in read contents (%s)", GetID(), err.message().c_str());
			Stop();
		}
	}
	void OnWriteHeader  (const system::error_code& err){
		if (!err){
			Message::Extra("session %d: response header sent", GetID());

			if(owner->txBuf.size()){
				// 送信コンテンツがあれば送信開始
				async_write(sock, buffer(&owner->txBuf[0], owner->txBuf.size()), boost::bind(&SessionBaseImpl::OnWriteContents, this, _1));
			}
			else{
				// 次のリクエストヘッダを受け付け
				async_read(sock, buffer(&owner->rxHeader, sizeof(RequestHeader)), boost::bind(&SessionBaseImpl::OnReadHeader, this, _1));
			}
		}
		else{
			Message::Error("session %d: error in write footer (%s)", GetID(), err.message().c_str());
			Stop();
		}
	}
	void OnWriteContents(const system::error_code& err){
		if (!err){
			Message::Extra("session %d: contents sent", GetID());
		
			// 次のリクエストヘッダを受け付け
			async_read(sock, buffer(&owner->rxHeader, sizeof(RequestHeader)), boost::bind(&SessionBaseImpl::OnReadHeader, this, _1));
		}
		else{
			Message::Error("session %d: error in write contents (%s)", GetID(), err.message().c_str());
			Stop();
		}
	}

public:
	int			 GetID(){ return id; }
	tcp::socket& GetSocket(){ return sock; }
	
	void Start(){
		Message::Out("session %d: started", GetID());

		// ヘッダ受信開始
		async_read(sock, buffer(&owner->rxHeader, sizeof(RequestHeader)), boost::bind(&SessionBaseImpl::OnReadHeader, this, _1));
	}

	void Stop(){
		owner->OnStop();
		if(sock.is_open())
			sock.close();
		Message::Out("session %d: stopped", GetID());
	}

	 SessionBaseImpl(int _id, io_service& ios):id(_id), ioService(ios), sock(ios){}
	~SessionBaseImpl(){}
};

class ServerBaseImpl{
protected:
	friend class ServerBase;

	ServerBase*		owner;

	/// boost::asio objects
	io_service		ioService;
	tcp::acceptor*	acceptor;

	/// thread to run io_service
	boost::thread	threadIoService;

	/// array of sessions
	Sessions		sessions;

	bool			running;

	/** internal callbacks **/
	void OnAccept(const system::error_code& error){
		if (!error){
			sessions.back()->impl->Start();

			// wait for another connection request
			StartSession();
		}
	}

	void OnStop(){
		// accept終了
		acceptor->close();
		// 走っているセッションを終了
		for(Sessions::iterator it = sessions.begin(); it != sessions.end(); it++)
			(*it)->impl->Stop();
	}

public:
	/// create new session
	void StartSession(){
		SessionBase* s = owner->CreateSession();
		s->impl = new SessionBaseImpl((int)sessions.size(), ioService);
		s->impl->owner = s;
		sessions.push_back(s);
		acceptor->async_accept(sessions.back()->impl->GetSocket(), boost::bind(&ServerBaseImpl::OnAccept, this, _1));
	}

	/// start running
	void Run(int port){
		if(running)
			return;
		// create endpoint
		tcp::endpoint ep(tcp::v4(), port);

		// create acceptor
		acceptor = new tcp::acceptor(ioService, ep);

		// create new session
		StartSession();

		// バックグラウンドでio_service始動
		threadIoService = boost::thread(boost::bind(&io_service::run, &ioService));

		Message::Out("server: listening on port %d", port);
		running = true;
	}

	/// stop
	void Stop(){
		if(!running)
			return;
		// 稼働中のセッションに停止要求
		ioService.post(boost::bind(&ServerBaseImpl::OnStop, this));
		
		// サービスの停止を待つ
		threadIoService.join();
		
		// セッションクリア
		sessions.clear();
		
		Message::Out("server: stopped");
		running = false;
	}

	ServerBaseImpl(){
	 	acceptor = 0;
		running  = false;
	}
	~ServerBaseImpl(){
		if(acceptor)
			delete acceptor;
	}
};

class ClientBaseImpl{
protected:
	friend class ClientBase;

	ClientBase*			owner;

	/// boost::asio objects
	io_service			ioService;
	io_service::work*	work;
	tcp::socket			sock;
	tcp::resolver		resolver;
	deadline_timer		connectTimer;
	deadline_timer		readTimer;
	deadline_timer		writeTimer;
	
	/// thread to run io_service
	boost::thread		threadIoService;

	typedef unsigned char		byte;
	
protected:
	/** internal callbacks **/
	void OnResolve(const system::error_code& err, tcp::resolver::iterator itNext){
		if (!err){
			TryNextEndpoint(itNext);
		}
		else{
			Message::Extra("client: resolve: %s", err.message().c_str());
			owner->connecting = false;
			owner->failed     = true;
			owner->evConnectCmpl.Set();
		}
	}
	void OnConnect(const system::error_code& err, tcp::resolver::iterator itNext){
		// タイマ停止
		connectTimer.cancel();

		if (!err){
			// 接続成功
			Message::Out("client: connection established");
			resolver.cancel();
			owner->connecting = false;
			owner->connected  = true;
			owner->evConnectCmpl.Set();
		}
		else if (itNext != tcp::resolver::iterator()){
			// 接続失敗したので次のendpointを試す
			sock.close();
			TryNextEndpoint(itNext);
		}
		else {
			// 全て失敗
			Message::Error("client: connection failed: %s", err.message().c_str());
			sock.close();
			owner->connecting = false;
			owner->failed     = true;
			owner->evConnectCmpl.Set();
			delete work;
		}
	}
	void OnConnectTimeout(const system::error_code& err, tcp::resolver::iterator itNext){
		if(!err){
			Message::Error("client: connection timeout");
			sock.close();
			if(itNext != tcp::resolver::iterator()){
				TryNextEndpoint(itNext);
			}
			else{
				owner->connecting = false;
				owner->failed     = true;
				owner->timedOut   = true;
				owner->evConnectCmpl.Set();
				delete work;
			}
		}
	}
	void OnReadTimeout(const system::error_code& err){
		if(!err){
			Message::Error("client: read operation timeout");
			sock.cancel();
			owner->reading  = false;
			owner->failed   = true;
			owner->timedOut = true;
			owner->evReadCmpl.Set();
		}
	}
	void OnWriteTimeout(const system::error_code& err){
		if(!err){
			Message::Error("client: write operation timeout");
			sock.cancel();
			owner->writing  = false;
			owner->failed   = true;
			owner->timedOut = true;
			owner->evWriteCmpl.Set();
		}
	}
	void OnReadHeader(const system::error_code& err, size_t n){
		readTimer.cancel();
		if(!err){
			Message::Extra("client: response header received");
			if(owner->rxHeader.sz){
				owner->rxBuf.Reset(owner->rxHeader.sz);
				owner->readBytesCmpl = 0;

				readTimer.expires_from_now(boost::posix_time::seconds(owner->readTimeout));
				readTimer.async_wait(boost::bind(&ClientBaseImpl::OnReadTimeout, this, _1));
				sock.async_read_some(buffer(&owner->rxBuf[0], owner->rxHeader.sz), boost::bind(&ClientBaseImpl::OnReadContents, this, _1, _2));
			}
			else{
				Message::Extra("client: read complete");
				owner->reading = false;
				owner->evReadCmpl.Set();
			}
		}
		else{
			Message::Error("client: read header failed: %s", err.message().c_str());
			owner->reading = false;
			owner->failed  = true;
			owner->evReadCmpl.Set();
		}
	}
	void OnWriteHeader(const system::error_code& err, size_t n){
		writeTimer.cancel();
		if(!err){
			Message::Extra("client: request header sent");
			if(owner->txHeader.sz){
				owner->writeBytesCmpl = 0;

				writeTimer.expires_from_now(boost::posix_time::seconds(owner->writeTimeout));
				writeTimer.async_wait(boost::bind(&ClientBaseImpl::OnWriteTimeout, this, _1));
				sock.async_write_some(buffer(&owner->txBuf[0], owner->txHeader.sz), boost::bind(&ClientBaseImpl::OnWriteContents, this, _1, _2));
			}
			else{
				Message::Extra("client: write complete");
				owner->writing = false;
				owner->evWriteCmpl.Set();
			}
		}
		else{
			Message::Error("client: write header failed: %s", err.message().c_str());
			owner->writing = false;
			owner->failed  = true;
			owner->evWriteCmpl.Set();
		}
	}
	void OnReadContents(const system::error_code& err, size_t n){
		// タイマ停止
		readTimer.cancel();
		if(!err){
			owner->readBytesCmpl += n;
			if(owner->readBytesCmpl == owner->rxHeader.sz){
				// 受信完了
				Message::Extra("client: read complete");
				owner->reading = false;
				owner->evReadCmpl.Set();
			}
			else{
				// 残りの受信リクエスト
				readTimer.expires_from_now(boost::posix_time::seconds(owner->readTimeout));
				readTimer.async_wait(boost::bind(&ClientBaseImpl::OnReadTimeout, this, _1));
				sock.async_read_some(buffer(&owner->rxBuf[0] + owner->readBytesCmpl, owner->rxHeader.sz - owner->readBytesCmpl),
					boost::bind(&ClientBaseImpl::OnReadContents, this, _1, _2));
			}
		}
		else{
			// 受信失敗(タイムアウトによるキャンセル含む)
			Message::Error("client: read contents failed: %s", err.message().c_str());
			owner->reading = false;
			owner->failed  = true;
			owner->evReadCmpl.Set();
		}
	}
	void OnWriteContents(const system::error_code& err, size_t n){
		// タイマ停止
		writeTimer.cancel();
		if(!err){
			owner->writeBytesCmpl += n;
			if(owner->writeBytesCmpl == owner->txHeader.sz){
				// 送信完了
				Message::Extra("client: write complete");
				owner->writing = false;
				owner->evWriteCmpl.Set();
			}
			else{
				// 残りの受信リクエスト
				writeTimer.expires_from_now(boost::posix_time::seconds(owner->writeTimeout));
				writeTimer.async_wait(boost::bind(&ClientBaseImpl::OnWriteTimeout, this, _1));
				sock.async_write_some(buffer(&owner->txBuf[0] + owner->writeBytesCmpl, owner->txHeader.sz - owner->writeBytesCmpl),
					boost::bind(&ClientBaseImpl::OnWriteContents, this, _1, _2));
			}
		}
		else{
			// 受信失敗(タイムアウトによるキャンセル含む)
			Message::Error("client: write contents failed: %s", err.message().c_str());
			owner->writing = false;
			owner->failed  = true;
			owner->evWriteCmpl.Set();
		}
	}
	void TryNextEndpoint(tcp::resolver::iterator it){
		// iteratorで指示されたendpointへの接続を試みる
		// 接続成功orタイムアウト内に失敗 => OnConnect
		// タイムアウト => OnTimeout
		tcp::endpoint ep = *it;
		tcp::resolver::iterator itNext = ++it;
		sock.async_connect(ep, boost::bind(&ClientBaseImpl::OnConnect, this, _1, itNext));

		connectTimer.expires_from_now(boost::posix_time::seconds(owner->connectTimeout));
		connectTimer.async_wait(boost::bind(&ClientBaseImpl::OnConnectTimeout, this, _1, itNext));
	}

public:
	void Connect(const char* host, const char* port){
		if(owner->connected){
			Message::Error("already connected");
			return;
		}
		if(owner->connecting){
			Message::Error("already attempting to connect");
			return;
		}
		try{ 
			owner->connecting = true;
			owner->failed     = false;
			owner->timedOut   = false;

			// 接続シーケンス開始（つづきはOnResolve）
			tcp::resolver::query query(host, port);
			resolver.async_resolve(query, boost::bind(&ClientBaseImpl::OnResolve, this, _1, _2));

			// バックグラウンドでio_service始動
			threadIoService.join();
			ioService.reset();
			// Disconnectするまでスレッドが終了しないようにする
			work = new io_service::work(ioService);
			threadIoService = boost::thread(boost::bind(&io_service::run, &ioService));
		}
		catch(std::exception& e){
			cerr << e.what() << endl;
		}
	}
	void Disconnect(){
		if(!owner->connected){
			Message::Error("already disconnected");
			return;
		}
		// バックグラウンドのスレッドにソケットを閉じるようにお願いする
		ioService.post(boost::bind(&tcp::socket::close, &sock));
		// そのままスレッドの終了を待つ
		delete work;
		threadIoService.join();

		Message::Out("client: disconnected");
		owner->connected  = false;
		owner->failed     = false;
		owner->timedOut   = false;
		owner->reading    = false;
		owner->writing    = false;
	}

	void Read(){
		// 受信中は新たに受信開始しない
		if(owner->reading){
			Message::Error("client: previous read transaction still ongoing");
			return;
		}
		owner->rxHeader = ResponseHeader();
		owner->rxBuf.Reset();

		owner->reading  = true;
		owner->failed   = false;
		owner->timedOut = false;

		// 受信タイムアウトタイマ
		readTimer.expires_from_now(boost::posix_time::seconds(owner->readTimeout));
		readTimer.async_wait(boost::bind(&ClientBaseImpl::OnReadTimeout, this, _1));

		// 受信リクエスト
		async_read(sock, buffer(&owner->rxHeader, sizeof(ResponseHeader)), boost::bind(&ClientBaseImpl::OnReadHeader, this, _1, _2));
	}

	void Write(){
		// 送信中は新たに送信開始しない
		if(owner->writing){
			Message::Error("client: previous write transaction still ongoing");
			return;
		}

		owner->writing  = true;
		owner->failed   = false;
		owner->timedOut = false;

		// 送信タイムアウトタイマ
		writeTimer.expires_from_now(boost::posix_time::seconds(owner->writeTimeout));
		writeTimer.async_wait(boost::bind(&ClientBaseImpl::OnWriteTimeout, this, _1));

		// 送信リクエスト
		owner->txHeader.sz = (uint16_t)owner->txBuf.size();
		async_write(sock, buffer(&owner->txHeader, sizeof(RequestHeader)), boost::bind(&ClientBaseImpl::OnWriteHeader, this, _1, _2));
	}
	
	ClientBaseImpl():sock(ioService), connectTimer(ioService), readTimer(ioService), writeTimer(ioService), resolver(ioService){
	}
	~ClientBaseImpl(){
	}
};

//-------------------------------------------------------------------------------------------------

SessionBase::SessionBase(){
}

SessionBase::~SessionBase(){
	delete impl;
}

int SessionBase::GetID(){
	return impl->GetID();
}

//-------------------------------------------------------------------------------------------------

ServerBase::ServerBase(){
	impl = new ServerBaseImpl();
	impl->owner = this;
}

ServerBase::~ServerBase(){
	delete impl;
}

SessionBase* ServerBase::CreateSession(){
	return new SessionBase();
}

void ServerBase::Run(int port){
	impl->Run(port);
}

void ServerBase::Stop(){
	impl->Stop();
}

//-------------------------------------------------------------------------------------------------

ClientBase::ClientBase(){
	connectTimeout	= 10;
	readTimeout		= 10;
	writeTimeout	= 10;

	connecting = false;
	connected  = false;
	reading	   = false;
	writing	   = false;
	failed     = false;
	timedOut   = false;

	evConnectCmpl.Create();
	evReadCmpl   .Create();
	evWriteCmpl  .Create();
	
	impl = new ClientBaseImpl();
	impl->owner = this;
}

ClientBase::~ClientBase(){
	delete impl;
}

void ClientBase::Connect(const char* host, const char* port){
	impl->Connect(host, port);
}
void ClientBase::Disconnect(){
	impl->Disconnect();
}

void ClientBase::Read(){
	impl->Read();
}

void ClientBase::Write(){
	impl->Write();
}

bool ClientBase::SendRequest(){
	Write();
	while(writing);
	if(failed)
		return false;
	Read();
	while(reading);
	if(failed)
		return false;
	return true;
}

//-------------------------------------------------------------------------------------------------

SceneSession::SceneSession(SceneServer* s){
	server = s;
}

int SceneSession::HandleRequest(){
	int id;
	int req = rxHeader.req;
	txBuf.Reset(0);
	if(req == SceneRequest::GetRoot){
		id = server->scene->GetRoot();
		txBuf << id;
	}
	else if(req == SceneRequest::GetParent){
		rxBuf >> id;
		int parId = server->scene->GetParent(id);
		txBuf << parId;
	}
	else if(req == SceneRequest::GetChildren){
		vector<int> children;
		rxBuf >> id;
		server->scene->GetChildren(id, children);
		txBuf.Write( (byte*)&children[0], (uint)(children.size() * sizeof(int)) );
	}
	else if(req == SceneRequest::GetName){
		rxBuf >> id;
		string name = server->scene->GetName(id);
		txBuf << name;
	}
	else if(req == SceneRequest::AddChild){
		int parId, childId;
		rxBuf >> parId >> childId;
		server->scene->AddChild(parId, childId);
	}
	else if(req == SceneRequest::CreateObject){
		int type;
		string name;
		rxBuf >> type >> name;
		id = server->scene->CreateObject(type, name, server->typedb);
		txBuf << id;
	}
	else if(req == SceneRequest::GetObjectType){
		rxBuf >> id;
		int type = server->scene->GetObjectType(id);
		txBuf << type;
	}
	else if(req == SceneRequest::AddLink){
		int srcId, destId;
		string name;
		rxBuf >> srcId >> destId >> name;
		server->scene->AddLink(srcId, destId, name);
	}
	/*else if(req == SceneRequest::GetPropertyArray){
		rxBuf >> num;
		scene->GetPropertyArray((int*)rxBuf.Head(), num, txBuf);
	}
	else if(req == SceneRequest::SetPropertyArray){
		rxBuf >> num;
		int* idArray = (int*)rxBuf.Head();
		rxBuf.Seek(num * sizeof(int));
		scene->SetPropertyArray(idArray, num, rxBuf);
	}*/
	return -1;
}

//-------------------------------------------------------------------------------------------------

SceneServer::SceneServer(){
	scene  = 0;
	typedb = 0;
}

void SceneServer::Set(SceneBase* s, TypeDB* db){
	scene  = s;
	typedb = db;
}

SessionBase* SceneServer::CreateSession(){
	return new SceneSession(this);
}

}
