#include <sbiksolver.h>
#include <sbikjointsync.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

static const real_t pi  = M_PI;
static const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointSync::PosCon::PosCon(IKJointSync* s, int _idx, const string& _name):sync(s), idx(_idx), Constraint(s->solver, 1, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
	AddSLink(sync->joint[0]->q_var[idx]);
	AddSLink(sync->joint[1]->q_var[idx]);
}
void IKJointSync::PosCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-sync->ratio[idx]);
	((SLink*)links[1])->SetCoef( 1.0);
}
void IKJointSync::PosCon::CalcDeviation(){
	y[0] = sync->joint[1]->q_var[idx]->val - sync->ratio[idx]*sync->joint[0]->q_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointSync::VelCon::VelCon(IKJointSync* s, int _idx, const string& _name):sync(s), idx(_idx), Constraint(s->solver, 1, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
	AddSLink(sync->joint[0]->qd_var[idx]);
	AddSLink(sync->joint[1]->qd_var[idx]);
}
void IKJointSync::VelCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-sync->ratio[idx]);
	((SLink*)links[1])->SetCoef( 1.0);
}
void IKJointSync::VelCon::CalcDeviation(){
	y[0] = sync->joint[1]->qd_var[idx]->val - sync->ratio[idx]*sync->joint[0]->qd_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointSync::AccCon::AccCon(IKJointSync* s, int _idx, const string& _name):sync(s), idx(_idx), Constraint(s->solver, 1, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
	AddSLink(sync->joint[0]->qdd_var[idx]);
	AddSLink(sync->joint[1]->qdd_var[idx]);
}
void IKJointSync::AccCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-sync->ratio[idx]);
	((SLink*)links[1])->SetCoef( 1.0);
}
void IKJointSync::AccCon::CalcDeviation(){
	y[0] = sync->joint[1]->qdd_var[idx]->val - sync->ratio[idx]*sync->joint[0]->qdd_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointSync::IKJointSync(IKSolver* _solver, IKJoint* _joint0, IKJoint* _joint1, const string& _name){
	name     = _name;
	solver   = _solver;
	joint[0] = _joint0;
	joint[1] = _joint1;
	
	for(int i = 0; i < 6; i++){
		enablePos[i] = false;
		enableVel[i] = false;
		enableAcc[i] = false;
	}

	posWeight = 1.0;
	velWeight = 1.0;
	accWeight = 1.0;
}

void IKJointSync::SetRatio(int i, real_t _ratio){ ratio[i] = _ratio; }

void IKJointSync::EnablePos(int i, bool on){ enablePos[i] = on; }
void IKJointSync::EnableVel(int i, bool on){ enableVel[i] = on; }
void IKJointSync::EnableAcc(int i, bool on){ enableAcc[i] = on; }

void IKJointSync::SetPosWeight(int i, real_t _weight){ posWeight = _weight; }
void IKJointSync::SetVelWeight(int i, real_t _weight){ velWeight = _weight; }
void IKJointSync::SetAccWeight(int i, real_t _weight){ accWeight = _weight; }

void IKJointSync::Init(){
}

void IKJointSync::AddVar(){
}

void IKJointSync::AddCon(){
	int ndof = std::min(joint[0]->ndof, joint[1]->ndof);
	for(int i = 0; i < ndof; i++){
		pos_con[i] = new PosCon(this, i, name + "_pos");
		vel_con[i] = new VelCon(this, i, name + "_vel");
		acc_con[i] = new AccCon(this, i, name + "_acc");
	}
}

void IKJointSync::Prepare(){
	int ndof = std::min(joint[0]->ndof, joint[1]->ndof);
	for(int i = 0; i < ndof; i++){
		pos_con[i]->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos[i]);
		vel_con[i]->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel[i]);
		acc_con[i]->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc[i]);

		pos_con[i]->weight[0] = posWeight;
		vel_con[i]->weight[0] = velWeight;
		acc_con[i]->weight[0] = accWeight;
	}
}

void IKJointSync::Finish(){
}

void IKJointSync::Update(){
}

void IKJointSync::Draw(GRRenderIf* render){
}

}
