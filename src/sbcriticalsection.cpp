#include <sbcriticalsection.h>

#ifdef _WIN32
# include <windows.h>
#endif

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class CriticalSectionImpl{
public:
#ifdef _WIN32
	CRITICAL_SECTION	cs;
#endif

	CriticalSectionImpl(uint spin){
#ifdef _WIN32
		InitializeCriticalSectionAndSpinCount(&cs, spin);
#endif
	}

	~CriticalSectionImpl(){
#ifdef _WIN32
		DeleteCriticalSection(&cs);
#endif
	}

	void Enter(){
#ifdef _WIN32
		EnterCriticalSection(&cs);
#endif
	}

	bool TryEnter(){
#ifdef _WIN32
		return (TryEnterCriticalSection(&cs) == TRUE);
#else
		return false;
#endif
	}

	void Leave(){
#ifdef _WIN32
		LeaveCriticalSection(&cs);
#endif
	}
};

///////////////////////////////////////////////////////////////////////////////////////////////////

CriticalSection::CriticalSection(){
	target    = 0;
	spinCount = 0;
	impl      = 0;
	Create();
}

CriticalSection::CriticalSection(CriticalSection* _target){
	target = _target;
	target->Enter();
}

CriticalSection::~CriticalSection(){
	if(target)
		target->Leave();
	else
		Delete();
}

void CriticalSection::Create(){
	if(!impl)
		impl = new CriticalSectionImpl(spinCount);
}

void CriticalSection::Delete(){
	if(impl){
		delete impl;
		impl = 0;
	}
}

void CriticalSection::Enter(){
	impl->Enter();
}

bool CriticalSection::TryEnter(){
	return impl->TryEnter();
}

void CriticalSection::Leave(){
	impl->Leave();
}

}
