#pragma once

#include <sbtypes.h>

namespace Scenebuilder{;

/**
	Windows Critical Sectionのラッパー
 */

class CriticalSectionImpl;

class CriticalSection{
private:
	CriticalSection(const CriticalSection& ):impl(0), target(0), spinCount(0){}	///< 誤って呼ばれないように

protected:
	CriticalSectionImpl* impl;
	CriticalSection* target;

public:
	uint	spinCount;			///< WaitForSingleObjectせずにループで待つ回数．

public:
	void Create  ();
	void Delete  ();
	void Enter   ();
	bool TryEnter();
	void Leave   ();
	
	 CriticalSection();

	 // コンストラクタでEnter，デストラクタでLeaveする
	 CriticalSection(CriticalSection* _target);

	~CriticalSection();
};

}
