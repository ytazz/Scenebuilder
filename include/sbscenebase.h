#ifndef SB_SCENEBASE_H
#define SB_SCENEBASE_H

#include <sbtypes.h>

#include <boost/array.hpp>
using namespace boost;

/** Common Scene Interface
	共通シーンインタフェース

	SceneBaseを継承するクラスに対して
	Builderによる構築支援や
	Synchronizerによる同期機能が利用できる
 **/

namespace Scenebuilder{;


/**
	Common Scene Interface
	- オブジェクトの作成，削除
	- 属性や状態の取得と設定

	- シーン中のオブジェクトはユニークなIDをもつ
	- 文字列の名前は単なる属性であるので重複してもよい
	- シーンは階層を持たない（サブシーンは無い）

	- シーン間の同期メカニズム
		- 最初に片方のシーンをもとに複製を作る
		  この際オリジナルとコピーのIDの対応をsynchronizerが記憶する
		  部分的な複製を作るときの範囲指定はシーン中の座標に基づいて行う
		  
 **/
class SceneBase{
public:
	/** get array of ids and types  **/
	virtual void	GetIDArray(vector<int>& ids);

	/** get object **/
	virtual bool	GetObject(int id, Descriptor* desc);
	/** create object **/
	virtual int		CreateObject(const Descriptor* desc);

	// ＊配列版があってもよい
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
	- Descriptorの配列からなるシーン
	- 単なるコンテナでありシミュレーションや描画などの機能は持たない
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
