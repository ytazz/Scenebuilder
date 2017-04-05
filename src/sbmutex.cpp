#include <sbmutex.h>
#include <windows.h>

namespace Scenebuilder{;

std::map<void*, void*>	Mutex::handles;

Mutex::Mutex(){

}

Mutex::Mutex(Mutex* _target){
	target = _target;

	if(target)
		target->Lock();
}

Mutex::~Mutex(){
	if(target)
		target->Unlock();
	else
		Close();
}

void* Mutex::GetHandle(){
	HandleDB::iterator it = handles.find(this);
	if(it != handles.end())
		return it->second;
	return 0;
}

bool Mutex::Create(){
	Close();

	// nameが指定されていれば名前つき，なければ名前なしミューテックスを作成
	void* h = (void*)CreateMutexA(0, 1, (name == "" ? (const char*)0 : (const char*)name));
	if(!h)
		return false;
	handles[this] = h;
	return true;
}

void Mutex::Close(){
	void* h = GetHandle();
	if(h)
		CloseHandle(h);
}

bool Mutex::Lock(uint timeout){
	void* h = GetHandle();
	if(!h){
		return Create();
	}
	else{
		int res = WaitForSingleObject(h, timeout);
		// 占有プロセスが解放せずに落ちた
		if(res == WAIT_ABANDONED)
			// 新しく作成
			return Create();
		// 取得成功
		if(res == WAIT_OBJECT_0)
			return true;
		// タイムアウト
		if(res == WAIT_TIMEOUT)
			return false;
		// エラー
		if(res == WAIT_FAILED)
			return false;
		return false;
	}
}

void Mutex::Unlock(){
	void* h = GetHandle();
	if(!h)
		return;
	ReleaseMutex(h);
}

}
