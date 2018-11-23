#include <sbevent.h>

#ifdef _WIN32
# include <windows.h>
#endif

namespace Scenebuilder{;

std::map<void*, void*>	Event::handles;

Event::Event(){

}

Event::~Event(){
//	Close();
}

void* Event::GetHandle(){
	HandleDB::iterator it = handles.find(this);
	if(it != handles.end())
		return it->second;
	return 0;
}


bool Event::Create(bool manual){
	Close();

	// nameが指定されていれば名前つき，なければ名前なしイベントを作成
	void* h = 0;
#ifdef _WIN32
	h = (void*)CreateEventA(0, manual, 0, (name.empty() ? 0 : name.c_str()));
#endif
	if(!h)
		return false;
	handles[this] = h;
	return true;
}

void Event::Close(){
	void* h = GetHandle();
#ifdef _WIN32
	if(h)
		CloseHandle(h);
#endif
}

bool Event::Wait(uint timeout){
	void* h = GetHandle();
	if(!h){
		Create();
		h = GetHandle();
	}
#ifdef _WIN32
	int res = WaitForSingleObject(h, timeout);
	// 取得成功
	if(res == WAIT_OBJECT_0)
		return true;
	// タイムアウト
	if(res == WAIT_TIMEOUT)
		return false;
	// エラー
	if(res == WAIT_FAILED)
		return false;
#endif
	return false;
}

int Event::Wait(Event** ev, uint num, uint timeout, bool wait_all){
#ifdef _WIN32
	vector<HANDLE>	handles;
	for(uint i = 0; i < num; i++){
		void* h = ev[i]->GetHandle();
		if(!h){
			ev[i]->Create();
			h = ev[i]->GetHandle();
		}
		handles.push_back(h);
	}

	uint res = WaitForMultipleObjects(num, &handles[0], wait_all, timeout);

	// イベント検知
	if(WAIT_OBJECT_0 <= res && res < WAIT_OBJECT_0 + num){
		if(wait_all)
			return 0;
		return res - WAIT_OBJECT_0;
	}
	// タイムアウト
	if(res == WAIT_TIMEOUT)
		return -1;
	// エラー
	if(res == WAIT_FAILED)
		return -1;
#endif
	return -1;
}

void Event::Set(){
	void* h = GetHandle();
	if(!h){
		Create();
		h = GetHandle();
	}
#ifdef _WIN32
	SetEvent(h);
#endif
}

bool Event::IsSet(){
	void* h = GetHandle();
	if(!h)
		Create();
	return Wait(0);
}

void Event::Reset(){
	void* h = GetHandle();
	if(!h){
		Create();
		h = GetHandle();
	}
#ifdef _WIN32
	ResetEvent(h);
#endif
}

}
