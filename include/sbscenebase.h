#ifndef SB_SCENEBASE_H
#define SB_SCENEBASE_H

#include <sbtypes.h>

#include <boost/array.hpp>
using namespace boost;

/** Common Scene Interface
	���ʃV�[���C���^�t�F�[�X

	SceneBase���p������N���X�ɑ΂���
	Builder�ɂ��\�z�x����
	Synchronizer�ɂ�铯���@�\�����p�ł���
 **/

namespace Scenebuilder{;


/**
	Common Scene Interface
	- �I�u�W�F�N�g�̍쐬�C�폜
	- �������Ԃ̎擾�Ɛݒ�

	- �V�[�����̃I�u�W�F�N�g�̓��j�[�N��ID������
	- ������̖��O�͒P�Ȃ鑮���ł���̂ŏd�����Ă��悢
	- �V�[���͊K�w�������Ȃ��i�T�u�V�[���͖����j

	- �V�[���Ԃ̓������J�j�Y��
		- �ŏ��ɕЕ��̃V�[�������Ƃɕ��������
		  ���̍ۃI���W�i���ƃR�s�[��ID�̑Ή���synchronizer���L������
		  �����I�ȕ��������Ƃ��͈͎̔w��̓V�[�����̍��W�Ɋ�Â��čs��
		  
 **/
class SceneBase{
public:
	/** get array of ids and types  **/
	virtual void	GetIDArray(vector<int>& ids);

	/** get object **/
	virtual bool	GetObject(int id, Descriptor* desc);
	/** create object **/
	virtual int		CreateObject(const Descriptor* desc);

	// ���z��ł������Ă��悢
	/** get state **/
	virtual bool	GetState(int id, State* state);
	/** get state **/
	virtual bool	SetState(int id, const State* state);

	/** @brief creates a new object
		@param name		name of the object
		@param desc		descriptor
		@return			returns -1 if failed, otherwise returns the id of the new object.
	 **/
	//virtual int		CreateShape(int type, UTString name, const ShapeDesc& desc){ return -1; }
	//virtual int		CreateBody (UTString name, const BodyDesc& desc) { return -1; }
	//virtual int		CreateJoint(int type, UTString name, const JointDesc& desc){ return -1; }
	//virtual bool	AssignShape (int bodyID, int shapeID, const Posed& pose){ return false; }
	//virtual bool	AssignSocket(int jointID, int bodyID, const Posed& pose){ return false; }
	//virtual bool	AssignPlug	(int jointID, int bodyID, const Posed& pose){ return false; }
	
	/**	@brief	gets the state of an object
	 **/
	//virtual bool	GetState(int id, State& state){ return false; }

	/** @brief	sets the state of an object
	 **/
	//virtual bool	SetState(int id, const State& state){ return false; }

};

/**	Scene
	- Descriptor�̔z�񂩂�Ȃ�V�[��
	- �P�Ȃ�R���e�i�ł���V�~�����[�V������`��Ȃǂ̋@�\�͎����Ȃ�
 **/
class Scene : public SceneBase{
public:
	typedef vector< UTRef<Descriptor> > Descriptors;
	Descriptors		descs;

public:
	/*virtual int		Create(Name name, const Desc& desc);
	virtual bool	Assign(int parentID, int childID);
	virtual bool	GetState(int id, State& state);
	virtual bool	SetState(int id, const State& state);*/

	/*int			NewID();
	Object*		GetObject(int id);
	Body*		GetBody  (int id);
	Shape*		GetShape (int id);
	Joint*		GetJoint (int id);

	virtual int		CreateBody	(UTString name, const BodyDesc& desc);
	virtual int		CreateShape	(int type, UTString name, const ShapeDesc& desc);
	virtual int		CreateJoint	(int type, UTString name, const JointDesc& desc);
	virtual bool	AssignShape	(int bodyID, int shapeID, const Posed& pose);
	virtual bool	AssignSocket(int jointID, int bodyID, const Posed& pose);
	virtual bool	AssignPlug	(int jointID, int bodyID, const Posed& pose);
	virtual void	SetGravity	(const Vec3d& gravity);
	virtual bool	GetState	(int id, State& state);
	virtual bool	SetState	(int id, const State& state);
	*/
};

}
#endif
