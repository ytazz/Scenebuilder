#ifndef SB_SYNC_H
#define SB_SYNC_H

#include <sbscene.h>

namespace Scenebuilder{;

/** Synchronizer
	- ���I�ɕω������鍀��:
	 - �I�u�W�F�N�g�̍쐬�ƍ폜	-> ID�Ή�
	 - �v���p�e�B�̕ω�			-> �^�C���X�^���v
	 - �X�e�[�g�̕ω�				-> �^�C���X�^���v
	 - �e�q�֌W�̕ω�			-> �l�����Ȃ�
	 - �����N�̍쐬�ƍ폜		-> �l�����Ȃ�

	- �����̎菇
	 - ID�Ή�������
	  - 1. �Е��ɃI�u�W�F�N�g������ID�Ή��������@-> �쐬
	  - 2. ID�Ή�������Е��ɃI�u�W�F�N�g�������@-> �폜
	  - 3. ID�Ή������藼���ɃI�u�W�F�N�g������
	   - �v���p�e�B�ƃX�e�[�g�̃^�C���X�^���v�ɕs��v�������
	     �Â������X�V
	 - 1�ō쐬���ꂽ�I�u�W�F�N�g�̐e�q�֌W�ƃ����N��ݒ肷��
	 
 **/
class Synchronizer{
protected:
	SceneBase*	scene[2];
	
	/** ID map
		- 
	 **/
	/// id -> {0,1}
	vector<byte>	alive[2];
	/// 
	vector<int>		idMap[2];
	/// array of live object ids
	vector<int>		liveId[2];

	Buffer			buf;
	
public:
	/** @brief	synchronize object tree
	 **/
	void SyncTree(Address& addr0, Address& addr1);

	/** @brief	synchronize properties
	 **/
	void SyncProperty();

	/** @brief	synchronize states
	 **/
	void SyncState();
	
	/** @brief register target scenes
	 **/
	void Set(SceneBase* s0, SceneBase* s1);

	Synchronizer();
	~Synchronizer();

};

}

#endif
