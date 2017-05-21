#pragma once

#include <sbtypes.h>
#include <sbvariable.h>
#include <sbconstraint.h>

#include <vector>
using namespace std;

#include <SprGraphics.h>
using namespace Spr;

namespace Scenebuilder{;

class IKSolver;

/// 剛体
class IKJoint : public UTRefCount{
public:
	struct Type{
		enum{
			Hinge,
			Slider,
			Balljoint,
			Fixjoint,
			LineToLine,
		};
	};

	class PosCon : public Constraint{
	public:
		IKJoint*  joint;
		int       dir;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		PosCon(IKJoint* jnt, int _dir);
	};
	class VelCon : public Constraint{
	public:
		IKJoint*  joint;
		int       dir;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		VelCon(IKJoint* jnt, int _dir);
	};
	class AccCon : public Constraint{
	public:
		IKJoint* joint;
		int      dir;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AccCon(IKJoint* jnt, int _dir);
	};

	IKSolver*  solver;
	IKBody*    sockBody;
	IKBody*    plugBody;
	IKBody*    rootBody;       ///< sock側とplug側の共通の親
	int        type;
	int        ndof;           ///< 関節の自由度
	bool       revolutive[3];  ///< 回転自由度ならtrue 直動自由度ならfalse
	
	vec3_t	sockPos;			///< 親剛体に対するソケットの位置と向き
	quat_t  sockOri;
	vec3_t	plugPos;			///< この剛体に対するプラグの位置と向き
	quat_t  plugOri;

	vec3_t  sockPosAbs;
	quat_t  sockOriAbs;
	vec3_t  plugPosAbs;
	quat_t  plugOriAbs;
	vec3_t  sockOffsetAbs;
	vec3_t  plugOffsetAbs;
	vec3_t  sockVelAbs;
	vec3_t  plugVelAbs;
	vec3_t  sockAccAbs;
	vec3_t  plugAccAbs;
	
	vec3_t	relPos;	            ///< ソケットに対するプラグの位置と向き
	quat_t  relOri;
	vec3_t  axis[3];
	vec3_t	Jv[3], Jv_abs[3];	///< ヤコビアン（並進）
	vec3_t  Jw[3], Jw_abs[3];	///< ヤコビアン（回転）

	vec3_t	pos;				///< グローバル座標上の位置と向き
	quat_t  ori;
	vec3_t  q;
	vec3_t  qd;
	vec3_t  qdd;
	vec3_t  tau;
	vec3_t  forceLocal;
	vec3_t  momentLocal;
	vec3_t  force;
	vec3_t  moment;

	vec3_t  q_ini;
	vec3_t  qd_ini;
	vec3_t  qdd_ini;
	vec3_t  tau_ini;

	vec3_t  q_limit[2];
	vec3_t  qd_limit[2];

	bool    q_lock  [3];
	bool    qd_lock [3];
	bool    qdd_lock[3];
	bool    tau_lock[3];

	SVar*   q_var  [3];             ///< 関節変位（ヒンジ，スライダ）
	SVar*   qd_var [3];
	SVar*   qdd_var[3];
	SVar*   force_var [3];
	SVar*   moment_var[3];

	PosCon* pos_con[3];
	VelCon* vel_con[3];
	AccCon* acc_con[3];
	
public:
	void CalcJacobian    ();
	void CalcRelativePose();
	
public:
	void SetSocketBody(IKBody* _sockBody);
	void SetPlugBody  (IKBody* _plugBody);

	/// 親リンクに対する相対的な位置と向きを設定
	void SetSocketPose(const pose_t& p);
	void SetPlugPose(const pose_t& p);

	void GetSocketPose(pose_t& p);
	void GetPlugPose(pose_t& p);

	/// 関節変数を取得
	real_t GetPos   (uint i);
	real_t GetVel   (uint i);
	real_t GetAcc   (uint i);
	real_t GetTorque(uint i);
	
	/// 関節変数を固定
	void LockPos   (uint i, bool lock = true);
	void LockVel   (uint i, bool lock = true);
	void LockAcc   (uint i, bool lock = true);
	void LockTorque(uint i, bool lock = true);
	
	/// IK計算の初期値を設定
	void SetInitialPos   (uint i, real_t _q  );
	void SetInitialVel   (uint i, real_t _qd );
	void SetInitialAcc   (uint i, real_t _qdd);
	void SetInitialTorque(uint i, real_t _tau);

	/// 関節角度の範囲拘束を設定
	void SetPosLimit(uint i, real_t lower, real_t upper);
	void SetVelLimit(uint i, real_t lower, real_t upper);

	void Init   ();
	void AddVar ();
	void AddCon ();
	void Prepare();
	void Finish ();
	void Update ();
	void Draw   (GRRenderIf* render);

	IKJoint(IKSolver* _solver, int _type);
};

}
