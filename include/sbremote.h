#pragma once

#include <sbtypes.h>
#include <sbbuffer.h>
#include <sbscene.h>
#include <sbevent.h>

#include <vector>
#include <cstdint>

/** ネットワーク通信のための基本機能の提供 **/

namespace Scenebuilder{;

struct RequestHeader{
	uint16_t req;		///< request
	uint16_t sz;		///< number of bytes in content body

	RequestHeader(){
		req = 0;
		sz  = 0;
	}
};

struct ResponseHeader{
	enum{
		Success,
		Error,
	};
	uint16_t req;		///< クライアントからのリクエスト
	uint16_t err;		///< error code
	uint16_t sz;		///< number of bytes in content body

	ResponseHeader(){
		req = 0;
		err = 0;
		sz  = 0;
	}
};

class SessionBaseImpl;
class ServerBaseImpl;
class ClientBaseImpl;

/** セッション：　クライアントとの接続単位
	・ヘッダを受信
	・ヘッダ内容に応じて受信あるいは送信
 **/
class SessionBase : public UTRefCount{
public:
	SessionBaseImpl*	impl;

	RequestHeader	rxHeader;
	ResponseHeader	txHeader;
	Buffer			rxBuf;
	Buffer			txBuf;

public:
	int		GetID();

	virtual int	 HandleRequest(){ return ResponseHeader::Error; }
	virtual void OnStop       (){}

	 SessionBase();
	~SessionBase();
};
typedef std::vector< UTRef<SessionBase> >	Sessions;

class ServerBase{
public:
	ServerBaseImpl*	impl;

public:
	/// start running
	void Run(int port);

	/// stop
	void Stop();

	virtual SessionBase* CreateSession();

	 ServerBase();
	~ServerBase();
};

class ClientBase{
public:
	ClientBaseImpl*	impl;

	RequestHeader	txHeader;
	ResponseHeader	rxHeader;
	Buffer			rxBuf;
	Buffer			txBuf;

	int		connectTimeout;		///< 接続タイムアウト
	int		readTimeout;		///< 受信タイムアウト
	int		writeTimeout;		///< 送信タイムアウト
	
	size_t	readBytesCmpl;		///< 受信済バイト数
	size_t	writeBytesCmpl;		///< 送信済バイト数

	// 状態フラグ
	bool	connecting;			///< 接続要求中
	bool	connected;			///< 接続中
	bool	reading;			///< 受信中
	bool	writing;			///< 送信中
	// 結果フラグ
	bool	failed;				///< 失敗した
	bool	timedOut;			///< タイムアウトした
	// イベント　失敗含む
	Event	evConnectCmpl;		//< 接続シーケンス完了
	Event	evReadCmpl;			//< 受信完了
	Event	evWriteCmpl;		//< 送信完了

public:
	/** @brief start connection procedure
		@param host
		@param port
	 **/
	void	Connect(const char* host, const char* port);

	/** @brief disconnect from server
	 **/
	void	Disconnect();

	/**	@brief read
		start asyncronous read operation
	 **/
	// レスポンス受信開始
	void Read();
	
	/** @brief write
		writes sz bytes of data and start asyncronous write operation.
	 **/
	// リクエスト送信開始
	void Write();

	// 送受信を同期実行
	bool SendRequest();

	ClientBase();
	virtual ~ClientBase();
};

//-------------------------------------------------------------------------------------------------

/** Scene同期用リクエスト **/
struct SceneRequest{
	enum{
		GetRoot,
		GetParent,
		GetChildren,
		GetName,
		SetName,
		AddChild,
		CreateObject,
		GetObjectType,
		AddLink,
		//GetPropertyArray,
		//SetPropertyArray,
		GetLivenessArray,
	};
};

class SceneServer;

class SceneSession : public SessionBase{
protected:
	SceneServer*	server;

public:
	virtual int	 HandleRequest();
	
	SceneSession(SceneServer* s);
};

class SceneServer : public ServerBase{
public:
	SceneBase*	scene;		///< reference to scene
	TypeDB*		typedb;

public:
	/// set scene
	void Set(SceneBase* s, TypeDB* db);

	virtual SessionBase* CreateSession();

	SceneServer();
};

}
