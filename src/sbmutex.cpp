#include <sbmutex.h>

#ifdef _WIN32
# include <windows.h>
#endif

namespace Scenebuilder{;

struct MutexHandle{
#if defined _WIN32
	HANDLE handle;
#elif defined __unix__
	pthread_mutex_t  mutex;
#endif
};

typedef std::map<void*, MutexHandle>	MutexHandles;
static MutexHandles  mutexHandles;		///< オブジェクトアドレスとミューテックスハンドルとの対応

MutexHandle* GetMutexHandle(Mutex* mutex){
	MutexHandles::iterator it = mutexHandles.find(mutex);
	if(it != mutexHandles.end())
		return &it->second;
	return 0;
}

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

bool Mutex::Create(){
	Close();

	// nameが指定されていれば名前つき，なければ名前なしミューテックスを作成
	MutexHandle h;
#if defined _WIN32
	h.handle = (void*)CreateMutexA(0, 1, (name == "" ? (const char*)0 : (const char*)name));
	if(!h.handle)
		return false;
#elif defined __unix__
	if(pthread_mutex_init(&h.mutex, 0))
		return false;
#endif
	mutexHandles[this] = h;
	return true;
}

void Mutex::Close(){
	MutexHandle* h = GetMutexHandle(this);
#if defined _WIN32
	if(h->handle)
		CloseHandle(h->handle);
#elif defined __unix__
	pthread_mutex_destroy(&h->mutex);
#endif
}

bool Mutex::Lock(uint timeout){
	MutexHandle* h = GetMutexHandle(this);
	if(!h){
		return Create();
	}
	else{
#if defined _WIN32
		int res = WaitForSingleObject(h->handle, timeout);
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
#elif defined __unix__
		if(pthread_mutex_lock(&h->mutex))
			return false;
		return true;
#endif
		return false;
	}
}

void Mutex::Unlock(){
	MutexHandle* h = GetMutexHandle(this);
	if(!h)
		return;
#if defined _WIN32
	ReleaseMutex(h->handle);
#elif defined __unix__
	pthread_mutex_unlock(&h->mutex);
#endif
}

}
