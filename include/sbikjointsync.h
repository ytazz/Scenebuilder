#pragma once

namespace Scenebuilder{;

class IKSolver;
class IKBody;

class IKJointSync : public UTRefCount{
public:
	class PosCon : public Constraint{
	public:
		IKJointSync* sync;
		int idx;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		PosCon(IKJointSync* s, int _idx, const string& _name);
	};
	class VelCon : public Constraint{
	public:
		IKJointSync* sync;
		int idx;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		VelCon(IKJointSync* s, int _idx, const string& _name);
	};
	class AccCon : public Constraint{
	public:
		IKJointSync* sync;
		int idx;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		AccCon(IKJointSync* s, int _idx, const string& _name);
	};
	
	string      name;
	IKSolver*   solver;
	IKJoint*	joint[2];
	
	PosCon*     pos_con[6];
	VelCon*     vel_con[6];
	AccCon*     acc_con[6];

	vec6_t  ratio;
	
	bool    enablePos[6];
	bool    enableVel[6];
	bool    enableAcc[6];

	real_t  posWeight;
	real_t  velWeight;
	real_t  accWeight;

public:
	void SetRatio(int i, real_t _ratio);
	
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

	IKJointSync(IKSolver* _solver, IKJoint* _joint0, IKJoint* _joint1, const string& _name);
};

}
