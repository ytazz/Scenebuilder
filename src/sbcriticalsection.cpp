#include <sbcriticalsection.h>
#include <windows.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

class CriticalSectionImpl{
public:
	CRITICAL_SECTION	cs;

	CriticalSectionImpl(uint spin){
		InitializeCriticalSectionAndSpinCount(&cs, spin);
	}

	~CriticalSectionImpl(){
		DeleteCriticalSection(&cs);
	}

	void Enter(){
		EnterCriticalSection(&cs);
	}

	bool TryEnter(){
		return (TryEnterCriticalSection(&cs) == TRUE);
	}

	void Leave(){
		LeaveCriticalSection(&cs);
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
