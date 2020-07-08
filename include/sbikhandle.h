#pragma once

namespace Scenebuilder{;

class IKSolver;
class IKBody;

class IKHandle : public UTRefCount{
public:
	class PosCon : public Constraint{
	public:
		IKHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		PosCon(IKHandle* h, const string& _name);
	};
	class OriCon : public Constraint{
	public:
		IKHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		OriCon(IKHandle* h, const string& _name);
	};
	class VelCon : public Constraint{
	public:
		IKHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		VelCon(IKHandle* h, const string& _name);
	};
	class AngvelCon : public Constraint{
	public:
		IKHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AngvelCon(IKHandle* h, const string& _name);
	};
	class AccCon : public Constraint{
	public:
		IKHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AccCon(IKHandle* h, const string& _name);
	};
	class AngaccCon : public Constraint{
	public:
		IKHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AngaccCon(IKHandle* h, const string& _name);
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

	void SetForce (const vec3_t& _force );
	void SetMoment(const vec3_t& _moment);

	void Init   ();
	void AddVar ();
	void AddCon ();
	void Prepare();
	void Finish ();
	void Update ();
	void Draw   (GRRenderIf* render);

	IKHandle(IKSolver* _solver, IKBody* _body, const string& _name);
};

class IKJointHandle : public UTRefCount{
public:
	class PosCon : public Constraint{
	public:
		IKJointHandle* handle;
		int idx;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		PosCon(IKJointHandle* h, int _idx, const string& _name);
	};
	class VelCon : public Constraint{
	public:
		IKJointHandle* handle;
		int idx;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		VelCon(IKJointHandle* h, int _idx, const string& _name);
	};
	class AccCon : public Constraint{
	public:
		IKJointHandle* handle;
		int idx;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AccCon(IKJointHandle* h, int _idx, const string& _name);
	};
	
	string      name;
	IKSolver*   solver;
	IKJoint*	joint;
	
	PosCon*     pos_con[6];
	VelCon*     vel_con[6];
	AccCon*     acc_con[6];

	vec6_t  pos;
	vec6_t  vel;
	vec6_t  acc;

	vec6_t  desPos;
	vec6_t  desVel;
	vec6_t  desAcc;

	bool    enablePos[6];
	bool    enableVel[6];
	bool    enableAcc[6];

public:
	void SetDesiredPos(int i, real_t pos);
	void SetDesiredVel(int i, real_t vel);
	void SetDesiredAcc(int i, real_t acc);
			
	void EnablePos(int i, bool on = true);
	void EnableVel(int i, bool on = true);
	void EnableAcc(int i, bool on = true);

	void Init   ();
	void AddVar ();
	void AddCon ();
	void Prepare();
	void Finish ();
	void Update ();

	void Draw(GRRenderIf* render);

	IKJointHandle(IKSolver* _solver, IKJoint* _joint, const string& _name);
};

class IKComHandleBase : public UTRefCount{
public:
	struct BodyInfo{
		IKBody*  body;
		real_t   massNorm;   ///< normalized mass
	};
	struct JointInfo{
		IKJoint* joint;
		vector<BodyInfo*> bodies;
	};

	vector<BodyInfo>   bodies;
	vector<JointInfo>  joints;
	IKBody*            root;

	string             name;
	IKSolver*          solver;

public:
	void AddBody   (IKBody* _body);
	void DeleteBody(IKBody* _body);

};

class IKComHandle : public IKComHandleBase{
public:
	class PosCon : public Constraint{
	public:
		IKComHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		PosCon(IKComHandle* h, const string& _name);
	};
	class VelCon : public Constraint{
	public:
		IKComHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		VelCon(IKComHandle* h, const string& _name);
	};
	class AccCon : public Constraint{
	public:
		IKComHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AccCon(IKComHandle* h, const string& _name);
	};

	PosCon*   pos_con;
	VelCon*   vel_con;
	AccCon*   acc_con;

	vec3_t    pos;
	vec3_t    vel;
	vec3_t    acc;

	vec3_t    comPosAbs;
	vec3_t    comVelAbs;
	vec3_t    comAccAbs;

	vec3_t    desPos;
	vec3_t    desVel;
	vec3_t    desAcc;

	bool      enablePos;
	bool      enableVel;
	bool      enableAcc;

	real_t    totalMass;
	
public:
	void SetDesiredPos(const vec3_t& pos);
	void SetDesiredVel(const vec3_t& vel);
	void SetDesiredAcc(const vec3_t& acc);
	void GetDesiredPos(vec3_t& pos);
	void GetDesiredVel(vec3_t& vel);
	void GetDesiredAcc(vec3_t& acc);
	
	void GetCurrentPos(vec3_t& pos);
	void GetCurrentVel(vec3_t& vel);
	void GetCurrentAcc(vec3_t& acc);

	real_t GetTotalMass();
	
	void EnablePos(bool on = true);
	void EnableVel(bool on = true);
	void EnableAcc(bool on = true);

	void   Init   ();
	void   AddVar ();
	void   AddCon ();
	void   Prepare();
	void   Finish ();
	void   Update ();
	
	void Draw(GRRenderIf* render);

	IKComHandle(IKSolver* _solver, const string& _name);
};

class IKMomentumHandle : public IKComHandleBase{
public:
	class MomentumCon : public Constraint{
	public:
		IKMomentumHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		MomentumCon(IKMomentumHandle* h, const string& _name);
	};
	
	MomentumCon*   mom_con;
	
	vec3_t    mom;
	vec3_t    desMom;
	
	bool      enable;
	
public:
	void SetDesiredMomentum(const vec3_t& _mom);
	void GetDesiredMomentum(vec3_t& _mom);
	
	void GetCurrentMomentum(vec3_t& _mom);
	
	void Enable(bool on = true);
	
	void   Init   ();
	void   AddVar ();
	void   AddCon ();
	void   Prepare();
	void   Finish ();
	void   Update ();
	
	void Draw(GRRenderIf* render);

	IKMomentumHandle(IKSolver* _solver, const string& _name);
};

}
