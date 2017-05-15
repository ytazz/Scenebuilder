﻿#pragma once

#include <sbtypes.h>
#include <sbvariable.h>
#include <sbconstraint.h>

#include <vector>
using namespace std;

#include <SprGraphics.h>
using namespace Spr;

namespace Scenebuilder{;

class IKSolver;
class IKJoint;

/// 剛体
class IKBody : public UTRefCount{
public:
	class ForceCon : public Constraint{
	public:
		IKBody*  body;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		ForceCon(IKBody* b);
	};

	class MomentCon : public Constraint{
	public:
		IKBody*  body;
	public:
		virtual void CalcCoef();
		virtual void CalcDeviation();
		MomentCon(IKBody* b);
	};

	IKSolver*           solver;
	IKBody*				parBody;
	IKJoint*            parJoint;
	vector< IKBody*  >  children;
	vector< IKJoint* >  joints;
	vector< IKHandle*>  handles;
	bool                ready;
	
	real_t  mass;
	vec3_t  center;
	mat3_t  inertia;

	vec3_t  centerPosAbs;
	vec3_t  centerVelAbs;
	vec3_t  centerAccAbs;

	vec3_t  pos;
	quat_t  ori;
	vec3_t  vel;
	vec3_t  angvel;
	vec3_t  acc;
	vec3_t  angacc;
	vec3_t  force;
	vec3_t  moment;
	
	V3Var*  pos_var;
	QVar*   ori_var;
	V3Var*  vel_var;
	V3Var*  angvel_var;
	V3Var*  acc_var;
	V3Var*  angacc_var;

	ForceCon*   force_con;
	MomentCon*  moment_con;

	vec3_t  cv;
	vec3_t  cw;

public:
	void    Init   ();
	void    AddVar ();
	void    AddCon ();
	void	Prepare();
	void    Finish ();
	void    Update ();
	//void	CompFK ();

public:
	IKBody*	 GetParent();
	void     SetParent(IKBody* par, IKJoint* _joint);
	IKJoint* GetJoint();

	void     SetMass(real_t m);
	real_t   GetMass();

	void     SetCenter(const vec3_t& c);
	void     GetCenter(vec3_t& c);

	/// 位置と向きの取得
	void SetPose(const pose_t& p);
	void GetPose(pose_t& p);

	void Draw(GRRenderIf* render);

	IKBody(IKSolver* _solver);
};

}