#pragma once

#include <sbsolver.h>
#include <sbconverter.h>

#include <SprGraphics.h>
using namespace Spr;

namespace Scenebuilder{;

class IKBody;
class IKJoint;
class IKHandle;
class IKComHandle;
typedef vector< UTRef<IKBody     > > IKBodyRefs;
typedef vector< UTRef<IKJoint    > > IKJointRefs;
typedef vector< UTRef<IKHandle   > > IKHandleRefs;
typedef vector< UTRef<IKComHandle> > IKComHandleRefs;


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
	Color   velColor;
	Color   accColor;
	Color   forceColor;
	Color   handleColor;
	float   velScale;
	float   accScale;
	float   forceScale;

	int     mode;
	bool    ready;

	int     timeInit;
	int     timePrepare;
	int     timeFinish;
	int     timeUpdate;
	
	IKBodyRefs       ikBodies;
	IKJointRefs      ikJoints;
	IKHandleRefs     ikHandles;
	IKComHandleRefs  ikComHandles;

	vec3_t  forceTotal;
	vec3_t  momentTotal;

public:
	
	void Init   ();
	void Prepare();
	void Finish ();
	void Update ();

public:
	/// Solverの仮想関数
	virtual real_t  CalcObjective();
	virtual void    CalcDirection();

public:
	/// 剛体を追加
	IKBody*		AddBody   ();
	void        DeleteBody(IKBody* body);

	///
	IKJoint*    AddJoint   (int _type);
	void        DeleteJoint(IKJoint* joint);

	/// 拘束を追加
	IKHandle*   AddHandle   (IKBody* sockBody);
	void        DeleteHandle(IKHandle* handle);

	///
	IKComHandle*  AddComHandle   ();
	void          DeleteComHandle(IKComHandle* handle);

	/** 逆キネ計算を行う
	 **/
	void	CompPosIK  ();
	void	CompVelIK  ();
	void	CompAccIK  ();
	void	CompForceIK();
	
	/** 順キネ計算を行う
		- 関節角度を所与として，根から末端に向かい剛体の位置と向きを求める
	 **/
	//void	CompFK();

	/// 描画
	void Draw(GRRenderIf* render);

	IKSolver();
};

}
