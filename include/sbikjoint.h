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

///
class IKJointBase{
public:
	string     name;
	IKSolver*  solver;
	IKBody*    sockBody;
	IKBody*    plugBody;
	int        type;
	int        ndof;            ///< 関節の自由度	
	
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
	
	vec3_t	pos;				///< グローバル座標上の位置と向き
	quat_t  ori;
	
	vec3_t  axis[6];

	vec3_t  forceLocal;
	vec3_t  momentLocal;
	vec3_t  force;
	vec3_t  moment;

	SVar*   force_var [6];
	SVar*   moment_var[6];

public:
	void SetSocketBody(IKBody* _sockBody);
	void SetPlugBody  (IKBody* _plugBody);

	/// 親リンクに対する相対的な位置と向きを設定
	void SetSocketPose(const pose_t& p);
	void SetPlugPose(const pose_t& p);

	void GetSocketPose(pose_t& p);
	void GetPlugPose(pose_t& p);

	virtual void Init   ();
	virtual void AddVar ();
	virtual void AddCon ();
	virtual void Reset  (){}
	virtual void Prepare();
	virtual void Finish ();
	virtual void Limit  (){}
	virtual void Update ();
	virtual void Draw   (GRRenderIf* render);

	IKJointBase(IKSolver* _solver, int _type, const string& _name);
};

/// 拘束
class IKMate : public UTRefCount, public IKJointBase{
public:
	struct Type{
		enum{
			PointToPoint,
			PointToLine,
			PointToPlane,
			Distance,
		};
	};

	class ConBase : public Constraint{
	public:
		IKMate*  mate;
	public:
		virtual void CalcCoef();
		ConBase(IKMate* _mate, const string& _name);
	};
	class PosCon : public ConBase{
	public:
		virtual void CalcDeviation();
		virtual void Project(real_t& l, uint k);
		PosCon(IKMate* _mate, const string& _name);
	};
	class VelCon : public ConBase{
	public:
		virtual void CalcDeviation();
		VelCon(IKMate* _mate, const string& _name);
	};
	class AccCon : public ConBase{
	public:
		virtual void CalcDeviation();
		AccCon(IKMate* _mate, const string& _name);
	};

public:
	real_t distance;
	vec3_t rangeMin;
	vec3_t rangeMax;

	IKBody*    rootBody;       ///< sock側とplug側の共通の親

	vec3_t pos_diff;
	vec3_t vel_diff;
	vec3_t acc_diff;

	PosCon* pos_con;
	VelCon* vel_con;
	AccCon* acc_con;

public:
	virtual void Init   ();
	virtual void AddVar ();
	virtual void AddCon ();
	virtual void Prepare();
	virtual void Finish ();
	virtual void Update ();
	virtual void Draw   (GRRenderIf* render);

	IKMate(IKSolver* _solver, int _type, const string& _name);
};

/// 関節
class IKJoint : public UTRefCount, public IKJointBase{
public:
	struct Type{
		enum{
			Hinge,
			Slider,
			Universaljoint,
			Balljoint,
			Freejoint,
			Fixjoint
		};
	};

	vec3_t	relPos;	            ///< ソケットに対するプラグの位置と向き
	quat_t  relOri;
	vec3_t	Jv[6], Jv_abs[6];	///< ヤコビアン（並進）
	vec3_t  Jw[6], Jw_abs[6];	///< ヤコビアン（回転）

	vec3_t  q;
	vec3_t  qd;
	vec3_t  qdd;
	vec3_t  tau;
	
	vec3_t  q_ini;
	vec3_t  qd_ini;
	vec3_t  qdd_ini;
	vec3_t  tau_ini;

	vec6_t  q_limit[2];
	vec6_t  qd_limit[2];

	bool    q_lock  [6];
	bool    qd_lock [6];
	bool    qdd_lock[6];
	bool    tau_lock[6];

	SVar*   q_var  [6];             ///< 関節変位（ヒンジ，スライダ）
	SVar*   qd_var [6];
	SVar*   qdd_var[6];
	
public:
	void CalcJacobian    ();
	void CalcRelativePose();
	
public:
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

	virtual void Init   ();
	virtual void AddVar ();
	virtual void AddCon ();
	virtual void Reset  ();
	virtual void Prepare();
	virtual void Finish ();
	virtual void Limit  ();
	virtual void Update ();
	virtual void Draw   (GRRenderIf* render);

	IKJoint(IKSolver* _solver, int _type, const string& _name);
};


/*
	checks complex kinematic constraints
	checking only (returns ok or not)
 */
class IKLimit : public UTRefCount, public IKJointBase{
public:
	struct Type{
		enum{
			Conic,
		};
	};

	real_t  angle;
	
public:
	virtual void Update ();
	
	real_t  CalcError();
	
public:	
	IKLimit(IKSolver* _solver, int _type, const string& _name);

};

}
