#pragma once

namespace Scenebuilder{;

class IKSolver;
class IKBody;

class IKBodyHandle : public UTRefCount{
public:
	class PosCon : public Constraint{
	public:
		IKBodyHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		PosCon(IKBodyHandle* h, const string& _name);
	};
	class OriCon : public Constraint{
	public:
		IKBodyHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		OriCon(IKBodyHandle* h, const string& _name);
	};
	class VelCon : public Constraint{
	public:
		IKBodyHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		VelCon(IKBodyHandle* h, const string& _name);
	};
	class AngvelCon : public Constraint{
	public:
		IKBodyHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AngvelCon(IKBodyHandle* h, const string& _name);
	};
	class AccCon : public Constraint{
	public:
		IKBodyHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AccCon(IKBodyHandle* h, const string& _name);
	};
	class AngaccCon : public Constraint{
	public:
		IKBodyHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AngaccCon(IKBodyHandle* h, const string& _name);
	};

	string      name;
	IKSolver*   solver;
	IKBody*	    body;         ///< エンドボディ（ハンドルを取り付けるボディ）
	
	PosCon*     pos_con;
	OriCon*     ori_con;
	VelCon*     vel_con;
	AngvelCon*  angvel_con;
	AccCon*     acc_con;
	AngaccCon*  angacc_con;

	vec3_t  pos;
	quat_t  ori;
	vec3_t  vel;
	vec3_t  angvel;
	vec3_t  acc;
	vec3_t  angacc;
	vec3_t  force;
	vec3_t  moment;

	vec3_t	sockPos;     ///< エンドボディから見たハンドルの位置と向き
	quat_t  sockOri;
	vec3_t  sockPosAbs;
	quat_t  sockOriAbs;
	vec3_t  sockOffsetAbs;
	vec3_t  sockVelAbs;
	vec3_t  sockAngvelAbs;
	vec3_t  sockAccAbs;
	vec3_t  sockAngaccAbs;

	vec3_t  desPos;
	quat_t  desOri;
	vec3_t  desVel;
	vec3_t  desAngvel;
	vec3_t  desAcc;
	vec3_t  desAngacc;

	bool    enablePos;
	bool    enableOri;
	bool    enableVel;
	bool    enableAngvel;
	bool    enableAcc;
	bool    enableAngacc;

	vec3_t  posWeight;
	vec3_t  oriWeight;
	vec3_t  velWeight;
	vec3_t  angvelWeight;
	vec3_t  accWeight;
	vec3_t  angaccWeight;

public:
	void SetSocketPose(const pose_t& p);
	void GetSocketPose(pose_t& p);
	
	void SetDesiredPos   (const vec3_t& pos   );
	void SetDesiredOri   (const quat_t& ori   );
	void SetDesiredVel   (const vec3_t& vel   );
	void SetDesiredAngvel(const vec3_t& angvel);
	void SetDesiredAcc   (const vec3_t& acc   );
	void SetDesiredAngacc(const vec3_t& angacc);
	
	void GetDesiredPos   (vec3_t& _pos   );
	void GetDesiredOri   (quat_t& _ori   );
	void GetDesiredVel   (vec3_t& _vel   );
	void GetDesiredAngvel(vec3_t& _angvel);
	void GetDesiredAcc   (vec3_t& _acc   );
	void GetDesiredAngacc(vec3_t& _angacc);
	
	void GetCurrentPos   (vec3_t& _pos   );
	void GetCurrentOri   (quat_t& _ori   );
	void GetCurrentVel   (vec3_t& _vel   );
	void GetCurrentAngvel(vec3_t& _angvel);
	void GetCurrentAcc(   vec3_t& _acc   );
	void GetCurrentAngacc(vec3_t& _angacc);
	
	void EnablePos   (bool on = true);
	void EnableOri   (bool on = true);
	void EnableVel   (bool on = true);
	void EnableAngvel(bool on = true);
	void EnableAcc   (bool on = true);
	void EnableAngacc(bool on = true);

	void SetPosWeight   (vec3_t weight);
	void SetOriWeight   (vec3_t weight);
	void SetVelWeight   (vec3_t weight);
	void SetAngvelWeight(vec3_t weight);
	void SetAccWeight   (vec3_t weight);
	void SetAngaccWeight(vec3_t weight);

	void SetForce (const vec3_t& _force );
	void SetMoment(const vec3_t& _moment);

	void Init   ();
	void AddVar ();
	void AddCon ();
	void Prepare();
	void Finish ();
	void Update ();
	void Draw   (GRRenderIf* render);

	IKBodyHandle(IKSolver* _solver, IKBody* _body, const string& _name);
};

}
