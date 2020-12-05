#pragma once

namespace Scenebuilder{;

class IKSolver;
class IKBody;

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

	real_t  posWeight;
	real_t  velWeight;
	real_t  accWeight;

public:
	void SetDesiredPos(int i, real_t pos);
	void SetDesiredVel(int i, real_t vel);
	void SetDesiredAcc(int i, real_t acc);

	void EnablePos(int i, bool on = true);
	void EnableVel(int i, bool on = true);
	void EnableAcc(int i, bool on = true);

	void SetPosWeight(int i, real_t weight);
	void SetVelWeight(int i, real_t weight);
	void SetAccWeight(int i, real_t weight);

	void Init   ();
	void AddVar ();
	void AddCon ();
	void Prepare();
	void Finish ();
	void Update ();

	void Draw(GRRenderIf* render);

	IKJointHandle(IKSolver* _solver, IKJoint* _joint, const string& _name);
};

}
