#pragma once

#include <sbtypes.h>

#include <map>

#pragma comment(lib, "winmm.lib")

namespace Scenebuilder{;

/// �^�C�}�R�[���o�b�N
class TimerCallback{
public:
	virtual void OnTimer() = 0;

};

class Timer{
public:
	typedef map<uint, Timer*>  IdMap;
	static IdMap idMap;

	uint			id;			///< �^�C�}ID
	uint			res;		///< �^�C�}����\
	TimerCallback*	callback;	///< �^�C�}�R�[���o�b�N�֐�
	
public:
	void SetCallback  (TimerCallback* _callback);
	void SetResolution(uint res);
	
	/** @brief �^�C�}�n��
		@param delay	�^�C�}����[ms]
	 */
	bool	Start(uint delay);

	/// �^�C�}��~
	void	Stop();

	/// �V�X�e������[ms]
	static uint	GetTime();

	 Timer();
	~Timer();
};

}
