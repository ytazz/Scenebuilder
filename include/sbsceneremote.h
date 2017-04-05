#pragma once

#include <sbscene.h>
#include <sbremote.h>

namespace Scenebuilder{;

/** SceneRemote
	ネットワークを介してアクセスするシーン

 **/
class SceneRemote : public SceneBase{
	ClientBase		client;
	RequestHeader	txHeader;
	ResponseHeader	rxHeader;
	Buffer			txBuf;
	Buffer			rxBuf;

	void SendRequest(int req);

public:
	/// TreeBase virtual functions
	virtual int			GetRoot    ();
	virtual int			GetParent  (int id);
	virtual void		GetChildren(int id, vector<int>& children);
	virtual string		GetName    (int id);
	virtual void        SetName    (int id, string name);
	virtual void		AddChild   (int parId, int childId);

	/// SceneBase virtual functions
	virtual int			CreateObject    (int type, const string& name, TypeDB* typedb);
	virtual int			GetObjectType   (int id);
	virtual void		AddLink         (int srcId, int destId);
	virtual void		RemoveLink      (int srcId, int destId);
	virtual int			GetLink         (int id, const string& name);
	virtual void		GetLinks        (int id, vector<int>& links, vector<string>& names);
	virtual Property*	GetProperty     (int id){ return 0; }
	//virtual void		GetPropertyArray(const int* idArray, size_t num, Buffer& buf);
	//virtual void		SetPropertyArray(const int* idArray, size_t num, Buffer& buf);
	virtual void		GetLivenessArray(Address& addr, vector<byte>& alive);
	
	void Connect(const char* host, const char* port);

	SceneRemote();
};

}
