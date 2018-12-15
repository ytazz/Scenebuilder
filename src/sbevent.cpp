#include <sbevent.h>

#if defined _WIN32
# include <windows.h>
#elif defined __unix__
# include <pthread.h>
#endif

namespace Scenebuilder{;

struct EventHandle{
#if defined _WIN32
	HANDLE  handle;
#elif defined __unix__
	pthread_mutex_t  mutex;
	pthread_cond_t   cond;
	bool value;
#endif
	bool manual;

	EventHandle(){
#if defined _WIN32
		handle = 0;
#elif defined __unix__
#endif
	}
};

typedef std::map<void*, EventHandle> EventHandles;
static EventHandles	eventHandles;

EventHandle* GetEventHandle(Event* ev){
	EventHandles::iterator it = eventHandles.find(ev);
	if(it != eventHandles.end())
		return &it->second;
	return 0;
}

Event::Event(){

}

Event::~Event(){
//	Close();
}

bool Event::Create(bool manual){
	Close();

	// nameが指定されていれば名前つき，なければ名前なしイベントを作成
	EventHandle h;
#if defined _WIN32
	h.handle = CreateEventA(0, manual, 0, (name.empty() ? 0 : name.c_str()));
	if(!h.handle)
		return false;
#elif defined __unix__
	if(pthread_cond_init(&h.cond, 0))
		return false;
	if(pthread_mutex_init(&h.mutex, 0))
		return false;
#endif
	h.manual = manual;
	eventHandles[this] = h;
	return true;
}

void Event::Close(){
	EventHandle* h = GetEventHandle(this);
	if(!h)
		return;
#if defined _WIN32
	if(h->handle)
		CloseHandle(h->handle);
#elif defined __unix__
	pthread_cond_destroy(&h.cond);
	pthread_mutex_destroy(&h.mutex);
#endif
}

bool Event::Wait(uint timeout){
	EventHandle* h = GetEventHandle(this);
	if(!h){
		Create();
		h = GetEventHandle(this);
	}
#if defined _WIN32
	int res = WaitForSingleObject(h->handle, timeout);
	// 取得成功
	if(res == WAIT_OBJECT_0)
		return true;
	// タイムアウト
	if(res == WAIT_TIMEOUT)
		return false;
	// エラー
	if(res == WAIT_FAILED)
		return false;
#elif defined __unix__
	const int _1e6 = 1000000;
	const int _1e9 = 1000000000;
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_nsec += _1e6 * timeout;
	if(ts.tv_nsec >= _1e9){
		ts.tv_sec++;
		ts.tv_nsec -= _1e9;
	}
	
	if(pthread_mutex_lock(&h->mutex))
		return false;
	if(pthread_cond_timedwait(&h->cond, &h->mutex, &ts))
		return false;

	return true;
#endif
	return false;
}

int Event::Wait(Event** ev, uint num, uint timeout, bool wait_all){
#ifdef _WIN32
	vector<HANDLE>	handles;
	for(uint i = 0; i < num; i++){
		EventHandle* h = GetEventHandle(ev[i]);
		if(!h){
			ev[i]->Create();
			h = GetEventHandle(ev[i]);
		}
		handles.push_back(h->handle);
	}

	uint res = WaitForMultipleObjects(num, &handles[0], wait_all, timeout);

	// event detected
	if(WAIT_OBJECT_0 <= res && res < WAIT_OBJECT_0 + num){
		if(wait_all)
			return 0;
		return res - WAIT_OBJECT_0;
	}
	// timed out
	if(res == WAIT_TIMEOUT)
		return -1;
	// error
	if(res == WAIT_FAILED)
		return -1;
#else
# error "this function is implemented for Win32 only"
#endif
	return -1;
}

void Event::Set(){
	EventHandle* h = GetEventHandle(this);
	if(!h){
		Create();
		h = GetEventHandle(this);
	}
#if defined _WIN32
	SetEvent(h->handle);
#elif defined __unix__
	h->value = true;
	pthread_cond_broadcast(&h->cond);
#endif

	for(EventGroup* gr : groups)
		gr->Set();
}

bool Event::IsSet(){
#if defined _WIN32
	return Wait(0);
#elif defined __unix__
	EventHandle* h = GetEventHandle(this);
	if(h)
		return h->value;
	return false;
#endif
}

void Event::Reset(){
	EventHandle* h = GetEventHandle(this);
	if(!h){
		Create();
		h = GetEventHandle(this);
	}
#if defined _WIN32
	ResetEvent(h->handle);
#elif defined __unix__
	h->value = false;
#endif
}

void EventGroup::Add(Event* ev){
	push_back(ev);
	ev->groups.push_back(this);
}

int EventGroup::Wait(uint timeout){
	for(Event* ev : *this)
		ev->Reset();

	if(!Event::Wait(timeout))
		return -1;

	for(int i = 0; i < (int)size(); i++){
		if(at(i)->IsSet())
			return i;
	}

	return -1;
}

}
