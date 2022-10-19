#include <sbiksolver.h>
#include <sbikjointhandle.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

static const real_t pi  = M_PI;
static const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::PosCon::PosCon(IKJointHandle* h, int _idx, const string& _name):handle(h), idx(_idx), Constraint(h->solver, 1, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
	AddSLink(handle->joint->q_var[idx]);
}
void IKJointHandle::PosCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::PosCon::CalcDeviation(){
	y[0] = handle->desPos[idx] - handle->joint->q_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::VelCon::VelCon(IKJointHandle* h, int _idx, const string& _name):handle(h), idx(_idx), Constraint(h->solver, 1, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
	AddSLink(handle->joint->qd_var[idx]);
}
void IKJointHandle::VelCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::VelCon::CalcDeviation(){
	real_t ep = handle->desPos[idx] - handle->joint->q_var[idx]->val;
	real_t w  = handle->solver->corrRate;
	real_t dt = handle->solver->dt;
	y[0] = (w*(ep/dt) + (1.0-w)*handle->desVel[idx]) - handle->joint->qd_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::AccCon::AccCon(IKJointHandle* h, int _idx, const string& _name):handle(h), idx(_idx), Constraint(h->solver, 1, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
	AddSLink(handle->joint->qdd_var[idx]);
}
void IKJointHandle::AccCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::AccCon::CalcDeviation(){
	y[0] = handle->desAcc[idx] - handle->joint->qdd_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::IKJointHandle(IKSolver* _solver, IKJoint* _joint, const string& _name){
	name   = _name;
	solver = _solver;
	joint  = _joint;
	
	for(int i = 0; i < 6; i++){
		enablePos[i] = false;
		enableVel[i] = false;
		enableAcc[i] = false;
	}

	posWeight = 1.0;
	velWeight = 1.0;
	accWeight = 1.0;
}

void IKJointHandle::SetDesiredPos(int i, real_t _pos){ desPos[i] = _pos; }
void IKJointHandle::SetDesiredAcc(int i, real_t _acc){ desAcc[i] = _acc; }
void IKJointHandle::SetDesiredVel(int i, real_t _vel){ desVel[i] = _vel; }

void IKJointHandle::EnablePos(int i, bool on){ enablePos[i] = on; }
void IKJointHandle::EnableVel(int i, bool on){ enableVel[i] = on; }
void IKJointHandle::EnableAcc(int i, bool on){ enableAcc[i] = on; }

void IKJointHandle::SetPosWeight(int i, real_t _weight){ posWeight = _weight; }
void IKJointHandle::SetVelWeight(int i, real_t _weight){ velWeight = _weight; }
void IKJointHandle::SetAccWeight(int i, real_t _weight){ accWeight = _weight; }

void IKJointHandle::Init(){
}

void IKJointHandle::AddVar(){
}

void IKJointHandle::AddCon(){
	for(int i = 0; i < joint->ndof; i++){
		pos_con[i] = new PosCon(this, i, name + "_pos");
		vel_con[i] = new VelCon(this, i, name + "_vel");
		acc_con[i] = new AccCon(this, i, name + "_acc");
	}
}

void IKJointHandle::Prepare(){
	for(int i = 0; i < joint->ndof; i++){
		pos_con[i]->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos[i]);
		vel_con[i]->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel[i]);
		acc_con[i]->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc[i]);

		pos_con[i]->weight[0] = posWeight;
		vel_con[i]->weight[0] = velWeight;
		acc_con[i]->weight[0] = accWeight;
	}
}

void IKJointHandle::Finish(){
}

void IKJointHandle::Update(){
}

void IKJointHandle::Draw(GRRenderIf* render){
}

}
