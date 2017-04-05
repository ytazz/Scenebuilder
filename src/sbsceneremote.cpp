#include <sbsceneremote.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

SceneRemote::SceneRemote(){

}

void SceneRemote::Connect(const char* host, const char* port){
	client.Connect(host, port);
}

void SceneRemote::SendRequest(int req){
	client.txHeader.req = req;
	client.SendRequest();
}

int SceneRemote::GetRoot(){
	txBuf.Reset(0);
	SendRequest(SceneRequest::GetRoot);
	int id;
	rxBuf >> id;
	return id;
}

int	SceneRemote::GetParent(int id){
	txBuf.Reset(0);
	txBuf << id;
	SendRequest(SceneRequest::GetParent);
	int parId;
	rxBuf >> parId;
	return parId;
}

void SceneRemote::GetChildren(int id, vector<int>& children){
	txBuf.Reset(0);
	txBuf << id;
	SendRequest(SceneRequest::GetChildren);
	
	size_t n = rxBuf.size() / sizeof(int);
	children.resize(n);
	rxBuf.Read( (byte*)&children[0], (uint)(children.size() * sizeof(int)) );
}

string SceneRemote::GetName(int id){
	txBuf.Reset(0);
	txBuf << id;
	SendRequest(SceneRequest::GetName);
	if(rxBuf.back() != '\0')
		throw SceneNetworkError();
	return string((const char*)rxBuf.Head());
}

void SceneRemote::SetName(int id, string name){
	txBuf.Reset(0);
	txBuf << id;
	txBuf << name;
	SendRequest(SceneRequest::SetName);
}

void SceneRemote::AddChild(int parId, int childId){
	txBuf.Reset(0);
	txBuf << parId << childId;
	SendRequest(SceneRequest::AddChild);
}

int SceneRemote::CreateObject(int type, const string& name, TypeDB* typedb){
	txBuf.Reset(0);
	txBuf << type;
	SendRequest(SceneRequest::CreateObject);
	int id;
	rxBuf >> id;
	return id;
}

int SceneRemote::GetObjectType(int id){
	txBuf.Reset(0);
	txBuf << id;
	SendRequest(SceneRequest::GetObjectType);
	int type;
	rxBuf >> type;
	return type;
}

void SceneRemote::AddLink(int srcId, int destId){
	txBuf.Reset(0);
	txBuf << srcId << destId;
	SendRequest(SceneRequest::AddLink);
}

void SceneRemote::RemoveLink(int srcId, int destId){

}

int	SceneRemote::GetLink(int id, const string& name){
	return 0;
}

void SceneRemote::GetLinks(int id, vector<int>& links, vector<string>& names){

}
	
/*	バッファのコピー処理は工夫次第で減らせそうだが
	ネットワーク通信の方が明らかなボトルネックであるので許容する
 */
/*void SceneRemote::GetPropertyArray(const int* idArray, size_t num, Buffer& buf){
	txBuf.Reset();
	txBuf << num;
	txBuf.Write((const byte*)idArray, num * sizeof(int));
	SendRequest(SceneRequest::GetPropertyArray);
	buf << rxBuf;
}

void SceneRemote::SetPropertyArray(const int* idArray, size_t num, Buffer& buf){
	txBuf.Reset();
	txBuf << num;
	txBuf.Write((const byte*)idArray, num * sizeof(int));
	txBuf << buf;
	SendRequest(SceneRequest::SetPropertyArray);
}*/

void SceneRemote::GetLivenessArray(Address& addr, vector<byte>& alive){
	txBuf.Reset();
	txBuf << addr.ToString();
	SendRequest(SceneRequest::GetLivenessArray);	
	alive.resize(rxBuf.size());
	rxBuf.Read(&alive[0], (uint)alive.size());
}

}
