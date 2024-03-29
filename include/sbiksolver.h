﻿#pragma once

#include <sbsolver.h>
#include <sbconverter.h>

#include <SprGraphics.h>
using namespace Spr;

namespace Scenebuilder{;

class IKBody;
class IKJoint;
class IKMate;
class IKLimit;
class IKBodyHandle;
class IKJointHandle;
class IKJointSync;
class IKComHandle;
typedef vector< UTRef<IKBody          > > IKBodyRefs;
typedef vector< UTRef<IKJoint         > > IKJointRefs;
typedef vector< UTRef<IKMate          > > IKMateRefs;
typedef vector< UTRef<IKLimit         > > IKLimitRefs;
typedef vector< UTRef<IKBodyHandle    > > IKBodyHandleRefs;
typedef vector< UTRef<IKJointHandle   > > IKJointHandleRefs;
typedef vector< UTRef<IKJointSync     > > IKJointSyncRefs;
typedef vector< UTRef<IKComHandle     > > IKComHandleRefs;

/** 逆運動学（inverse kinematics, IK）計算クラス
	- 基本的な解析
	 - 根が固定されたシリアルリンクを考える
	 - リンクの番号を根から末端にむかって0, 1, ...とする．
	 - リンクiとリンクi+1をつなぐ関節の番号をiとする
	 - 各関節の速度qd_jとリンクiの適当な点pにおける速度v_i，角速度w_iとの関係は

		v_i = \sum_j (eta_j qd_j) x (p - p_j)
		w_i = \sum_j (eta_j qd_j)

	   である．ただし
	    \sum_jはリンクiの上流にある関節に関する総和，
		pはリンクi上の適当な点，
		p_jは関節jの回転軸上の適当な点，
	    eta_jは関節jの軸方向
	   で，いずれもグローバル座標である．

	- より一般にツリー状リンクに対しても同様の関係が成り立つ．

	- 拘束解決エンジンはDiMPから流用

	- １ステップの更新
	 - 位置IKモードでは誤差を減少させる方向に変数を更新する．誤差を十分小さくするには何度かStepを呼ぶ必要がある
	 - 速度IKモードでは一度のStep呼び出しで拘束を満たす速度が求められる
	 
 **/

class IKSolver : public Solver{
public:
	struct Mode{
		enum{
			Pos,
			Vel,
			Acc,
			Force
		};
	};

	int     numIter;
	vec3_t  gravity;

	Color   bodyColor;
	Color   jointColor;
	Color   handleColor;
	Color   velColor;
	Color   angvelColor;
	Color   accColor;
	Color   angaccColor;
	Color   forceColor;
	Color   momentColor;
	
	float   velScale;
	float   angvelScale;
	float   accScale;
	float   angaccScale;
	float   forceScale;
	float   momentScale;

	bool    showBody;
	bool    showJoint;
	bool    showHandle;
	bool    showVel;
	bool    showAngvel;
	bool    showAcc;
	bool    showAngacc;
	bool    showForce;
	bool    showMoment;
	bool    showTorque;

	int     mode;
	//bool    ready;

	int     timeInit;
	int     timeReset;
	int     timePrepare;
	int     timeStep;
	int     timeFinish;
	int     timeUpdate;
	
	IKBodyRefs             ikBodies;
	IKJointRefs            ikJoints;
	IKMateRefs             ikMates;
	IKLimitRefs            ikLimits;
	IKBodyHandleRefs       ikBodyHandles;
	IKJointHandleRefs      ikJointHandles;
	IKJointSyncRefs        ikJointSyncs;
	IKComHandleRefs        ikComHandles;

	real_t  corrRate;
	real_t  dt;
	real_t  damping;

public:
	
	void Init   ();
	void Reset  ();
	void Prepare();
	void Finish ();
	void Update ();

public:
	/// Solverの仮想関数
	virtual real_t  CalcObjective();
	virtual void    CalcDirection();

public:
	/// 剛体を追加
	IKBody*		AddBody   (const string& name = "");
	void        DeleteBody(IKBody* body);

	///
	IKJoint*    AddJoint   (int _type, const string& name = "");
	void        DeleteJoint(IKJoint* joint);

	IKMate*     AddMate   (int _type, const string& name = "");
	void        DeleteMate(IKMate* mate);

	IKLimit*    AddLimit   (int _type, const string& name = "");
	void        DeleteLimit(IKLimit* limit);

	/// 拘束を追加
	IKBodyHandle*   AddBodyHandle   (IKBody* sockBody, const string& name = "");
	void            DeleteBodyHandle(IKBodyHandle* handle);

	IKJointHandle*   AddJointHandle   (IKJoint* joint, const string& name = "");
	void             DeleteJointHandle(IKJointHandle* handle);

	IKJointSync*     AddJointSync   (IKJoint* joint0, IKJoint* joint1, const string& name = "");
	void             DeleteJointSync(IKJointSync* sync);

	///
	IKComHandle*  AddComHandle   (const string& name = "");
	void          DeleteComHandle(IKComHandle* handle);

	/// position error correction rate of velocity IK
	void          SetCorrectionRate(real_t rate);

	/// time step used for integration and velocity IK error correction
	void          SetTimeStep(real_t dt);

	/// damping coefficient for variables
	void          SetDamping(real_t d);

	/** compute inverse kinematics
	 **/
	void	CompPosIK  ();
	void	CompVelIK  ();
	void	CompAccIK  ();
	void	CompForceIK();
	
	/** 順キネ計算を行う
		- 関節角度を所与として，根から末端に向かい剛体の位置と向きを求める
	 **/
	//void	CompFK();

	// 慣性行列を計算
	void    CompInertia(const vec3_t& origin, mat3_t& I);

	/// update position by integrating velocity
	void    Integrate();

	/// 描画
	void Draw(GRRenderIf* render);

	IKSolver();
};

}
