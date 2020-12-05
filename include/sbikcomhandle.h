#pragma once

namespace Scenebuilder{;

class IKSolver;
class IKBody;

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
	class MomentumCon : public Constraint{
	public:
		IKComHandle* handle;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		MomentumCon(IKComHandle* h, const string& _name);
	};

	PosCon*        pos_con;
	VelCon*        vel_con;
	AccCon*        acc_con;
	MomentumCon*   mom_con;
	
	vec3_t    pos;
	vec3_t    vel;
	vec3_t    acc;
	vec3_t    mom;

	vec3_t    comPosAbs;
	vec3_t    comVelAbs;
	vec3_t    comAccAbs;
	vec3_t    momAbs;
	mat3_t    comInertiaAbs;

	vec3_t    desPos;
	vec3_t    desVel;
	vec3_t    desAcc;
	vec3_t    desMom;

	bool      enablePos;
	bool      enableVel;
	bool      enableAcc;
	bool      enableMom;

	vec3_t    posWeight;
	vec3_t    velWeight;
	vec3_t    accWeight;
	vec3_t    momWeight;

	real_t    totalMass;
	
public:
	void SetDesiredPos     (const vec3_t& pos);
	void SetDesiredVel     (const vec3_t& vel);
	void SetDesiredAcc     (const vec3_t& acc);
	void SetDesiredMomentum(const vec3_t& mom);

	void GetDesiredPos     (vec3_t& pos);
	void GetDesiredVel     (vec3_t& vel);
	void GetDesiredAcc     (vec3_t& acc);
	void GetDesiredMomentum(vec3_t& mom);
	
	void GetCurrentPos     (vec3_t& pos);
	void GetCurrentVel     (vec3_t& vel);
	void GetCurrentAcc     (vec3_t& acc);
	void GetCurrentMomentum(vec3_t& mom);

	real_t GetTotalMass();
	
	void EnablePos     (bool on = true);
	void EnableVel     (bool on = true);
	void EnableAcc     (bool on = true);
	void EnableMomentum(bool on = true);

	void SetPosWeight     (vec3_t weight);
	void SetVelWeight     (vec3_t weight);
	void SetAccWeight     (vec3_t weight);
	void SetMomentumWeight(vec3_t weight);

	void   Init   ();
	void   AddVar ();
	void   AddCon ();
	void   Prepare();
	void   Finish ();
	void   Update ();
	
	void Draw(GRRenderIf* render);

	IKComHandle(IKSolver* _solver, const string& _name);
};

}
