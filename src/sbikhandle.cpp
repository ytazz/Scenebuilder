#include <sbiksolver.h>
#include <sbikhandle.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

static const real_t pi  = M_PI;

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::PosCon::PosCon(IKHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddC3Link(b->parJoint->q_var[i]);
			}
		}
		else{
			AddSLink (b->pos_var);
			AddX3Link(b->ori_var);
		}
	}
}
void IKHandle::PosCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((C3Link*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (handle->sockPosAbs - jnt->sockPosAbs)) );
			}
		}
		else{
			((SLink* )links[idx++])->SetCoef(-1.0);
			((X3Link*)links[idx++])->SetCoef(handle->sockPosAbs - b->pos_var->val);
		}
	}
}
void IKHandle::PosCon::CalcDeviation(){
	y = handle->desPos - handle->sockPosAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::OriCon::OriCon(IKHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddC3Link(b->parJoint->q_var[i]);
			}
		}
		else{
			AddSLink(b->ori_var);
		}
	}
}
void IKHandle::OriCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((C3Link*)links[idx++])->SetCoef(-1.0 * jnt->Jw_abs[i]);
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
		}
	}
}
void IKHandle::OriCon::CalcDeviation(){
	quat_t qerror = handle->sockOriAbs.Conjugated() * handle->desOri;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;

	y = handle->sockOriAbs * (theta * axis);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::VelCon::VelCon(IKHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < handle->body->parJoint->ndof; i++){
				AddC3Link(b->parJoint->qd_var[i]);
			}
		}
		else{
			AddSLink (b->vel_var   );
			AddX3Link(b->angvel_var);
		}
	}

}
void IKHandle::VelCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((C3Link*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (handle->sockPosAbs - jnt->sockPosAbs) ));
			}
		}
		else{
			((SLink* )links[idx++])->SetCoef(-1.0);
			((X3Link*)links[idx++])->SetCoef(handle->sockPosAbs - b->pos_var->val);
		}
	}
}
void IKHandle::VelCon::CalcDeviation(){
	vec3_t ep = handle->desPos - handle->sockPosAbs;
	real_t w  = handle->solver->corrRate;
	real_t dt = handle->solver->dt;
	y = (w*(ep/dt) + (1.0-w)*handle->desVel) - handle->sockVelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::AngvelCon::AngvelCon(IKHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddC3Link(b->parJoint->qd_var[i]);
			}
		}
		else{
			AddSLink(b->angvel_var);
		}
	}
}
void IKHandle::AngvelCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((C3Link*)links[idx++])->SetCoef(-1.0 * jnt->Jw_abs[i]);
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
		}
	}
}
void IKHandle::AngvelCon::CalcDeviation(){
	quat_t qerror = handle->sockOriAbs.Conjugated() * handle->desOri;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;

	vec3_t eq = handle->sockOriAbs * (theta * axis);
	real_t w  = handle->solver->corrRate;
	real_t dt = handle->solver->dt;
	
	y = (w*(eq/dt) + (1.0-w)*handle->desAngvel) - handle->sockAngvelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::AccCon::AccCon(IKHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddC3Link(b->parJoint->qdd_var[i]);
			}
		}
		else{
			AddSLink (b->acc_var   );
			AddX3Link(b->angacc_var);
		}
	}
}
void IKHandle::AccCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((C3Link*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (handle->sockPosAbs - jnt->sockPosAbs) ));
			}
		}
		else{
			((SLink* )links[idx++])->SetCoef(-1.0);
			((X3Link*)links[idx++])->SetCoef(handle->sockPosAbs - b->pos_var->val);
		}
	}
}
void IKHandle::AccCon::CalcDeviation(){
	y = handle->desAcc - handle->sockAccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::AngaccCon::AngaccCon(IKHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddC3Link(b->parJoint->qdd_var[i]);
			}
		}
		else{
			AddSLink(b->angacc_var);
		}
	}
}
void IKHandle::AngaccCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((C3Link*)links[idx++])->SetCoef(-1.0 * jnt->Jw_abs[i]);
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
		}
	}
}
void IKHandle::AngaccCon::CalcDeviation(){
	y = handle->desAngacc - handle->sockAngaccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::IKHandle(IKSolver* _solver, IKBody* _body, const string& _name){
	name   = _name;
	solver = _solver;
	body   = _body;
	
	enablePos    = false;
	enableOri    = false;
	enableVel    = false;
	enableAngvel = false;
	enableAcc    = false;
	enableAngacc = false;

	posWeight    = 1.0;
	oriWeight    = 1.0;
	velWeight    = 1.0;
	angvelWeight = 1.0;
	accWeight    = 1.0;
	angaccWeight = 1.0;
}

void IKHandle::SetSocketPose   (const pose_t& p){ sockPos = p.Pos(); sockOri = p.Ori(); }
void IKHandle::GetSocketPose   (      pose_t& p){ p.Pos() = sockPos; p.Ori() = sockOri; }

void IKHandle::GetCurrentPos   (vec3_t& _pos   ){ _pos    = pos;    }
void IKHandle::GetCurrentOri   (quat_t& _ori   ){ _ori    = ori;    }
void IKHandle::GetCurrentVel   (vec3_t& _vel   ){ _vel    = vel;    }
void IKHandle::GetCurrentAngvel(vec3_t& _angvel){ _angvel = angvel; }
void IKHandle::GetCurrentAcc   (vec3_t& _acc   ){ _acc    = acc;    }
void IKHandle::GetCurrentAngacc(vec3_t& _angacc){ _angacc = angacc; }

void IKHandle::SetDesiredPos   (const vec3_t& _pos   ){ desPos       = _pos;      }
void IKHandle::SetDesiredOri   (const quat_t& _ori   ){ desOri       = _ori;      }
void IKHandle::SetDesiredVel   (const vec3_t& _vel   ){ desVel       = _vel;      }
void IKHandle::SetDesiredAngvel(const vec3_t& _angvel){ desAngvel    = _angvel;   }
void IKHandle::SetDesiredAcc   (const vec3_t& _acc   ){ desAcc       = _acc;      }
void IKHandle::SetDesiredAngacc(const vec3_t& _angacc){ desAngvel    = _angacc;   }
void IKHandle::GetDesiredPos   (      vec3_t& _pos   ){ _pos         = desPos;    }
void IKHandle::GetDesiredOri   (      quat_t& _ori   ){ _ori         = desOri;    }
void IKHandle::GetDesiredVel   (      vec3_t& _vel   ){ _vel         = desVel;    }
void IKHandle::GetDesiredAngvel(      vec3_t& _angvel){ _angvel      = desAngvel; }
void IKHandle::GetDesiredAcc   (      vec3_t& _acc   ){ _acc         = desAcc;    }
void IKHandle::GetDesiredAngacc(      vec3_t& _angacc){ _angacc      = desAngacc; }

void IKHandle::EnablePos   (bool on){ enablePos    = on; }
void IKHandle::EnableOri   (bool on){ enableOri    = on; }
void IKHandle::EnableVel   (bool on){ enableVel    = on; }
void IKHandle::EnableAngvel(bool on){ enableAngvel = on; }
void IKHandle::EnableAcc   (bool on){ enableAcc    = on; }
void IKHandle::EnableAngacc(bool on){ enableAngacc = on; }

void IKHandle::SetPosWeight   (real_t weight){ posWeight    = weight; }
void IKHandle::SetOriWeight   (real_t weight){ oriWeight    = weight; }
void IKHandle::SetVelWeight   (real_t weight){ velWeight    = weight; }
void IKHandle::SetAngvelWeight(real_t weight){ angvelWeight = weight; }
void IKHandle::SetAccWeight   (real_t weight){ accWeight    = weight; }
void IKHandle::SetAngaccWeight(real_t weight){ angaccWeight = weight; }

void IKHandle::SetForce (const vec3_t& _force ){ force  = _force ; }
void IKHandle::SetMoment(const vec3_t& _moment){ moment = _moment; }

void IKHandle::Init(){

}

void IKHandle::AddVar(){
}

void IKHandle::AddCon(){
	pos_con    = new PosCon   (this, name + "_pos"   );
	ori_con    = new OriCon   (this, name + "_ori"   );
	vel_con    = new VelCon   (this, name + "_vel"   );
	angvel_con = new AngvelCon(this, name + "_angvel");
	acc_con    = new AccCon   (this, name + "_acc"   );
	angacc_con = new AngaccCon(this, name + "_angacc");
}

void IKHandle::Prepare(){
	pos_con   ->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos   );
	ori_con   ->enabled = (solver->mode == IKSolver::Mode::Pos && enableOri   );
	vel_con   ->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel   );
	angvel_con->enabled = (solver->mode == IKSolver::Mode::Vel && enableAngvel);
	acc_con   ->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc   );
	angacc_con->enabled = (solver->mode == IKSolver::Mode::Acc && enableAngacc);

	pos_con   ->weight = posWeight   ;
	ori_con   ->weight = oriWeight   ;
	vel_con   ->weight = velWeight   ;
	angvel_con->weight = angvelWeight;
	acc_con   ->weight = accWeight   ;
	angacc_con->weight = angaccWeight;
}

void IKHandle::Finish(){
	if(solver->mode == IKSolver::Mode::Pos){
		pos    = sockPosAbs;
		ori    = sockOriAbs;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		vel    = sockVelAbs;
		angvel = sockAngvelAbs;
	}
	if(solver->mode == IKSolver::Mode::Acc){
		acc    = sockAccAbs;
		angacc = sockAngaccAbs;
	}
	if(solver->mode == IKSolver::Mode::Force){
	}
}

void IKHandle::Update(){
	if(solver->mode == IKSolver::Mode::Pos){
		sockOffsetAbs = body->ori_var->val * sockPos;
		sockPosAbs    = body->pos_var->val + sockOffsetAbs;
		sockOriAbs    = body->ori_var->val * sockOri;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		sockVelAbs    = body->vel_var->val + body->angvel_var->val % sockOffsetAbs;
		sockAngvelAbs = body->angvel_var->val;
	}
	if(solver->mode == IKSolver::Mode::Acc){
		sockAccAbs    = body->acc_var->val + body->angacc_var->val % sockOffsetAbs + body->angvel_var->val % (body->angvel_var->val % sockOffsetAbs);
		sockAngaccAbs = body->angacc_var->val;
	}
}

void IKHandle::Draw(GRRenderIf* render){
	Vec3f p0, p1;

	if(solver->showHandle){
		glColor4fv((float*)solver->handleColor.rgba);
		render->SetPointSize(5.0f);
	
		p0 = pos;
		render->DrawPoint(p0);

		p1 = desPos;
		render->DrawPoint(p0);

		p0 = body->pos;
		p1 = pos;
		render->DrawLine(p0, p1);
	}
}	

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::PosCon::PosCon(IKJointHandle* h, int _idx, const string& _name):handle(h), idx(_idx), Constraint(h->solver, 1, ID(0, 0, 0, _name), 1.0){
	AddSLink(handle->joint->q_var[idx]);
}
void IKJointHandle::PosCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::PosCon::CalcDeviation(){
	y[0] = handle->desPos[idx] - handle->joint->q_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::VelCon::VelCon(IKJointHandle* h, int _idx, const string& _name):handle(h), idx(_idx), Constraint(h->solver, 1, ID(0, 0, 0, _name), 1.0){
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

IKJointHandle::AccCon::AccCon(IKJointHandle* h, int _idx, const string& _name):handle(h), idx(_idx), Constraint(h->solver, 1, ID(0, 0, 0, _name), 1.0){
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

void IKJointHandle::SetPosWeight(int i, real_t _weight){
	posWeight = _weight;
}

void IKJointHandle::SetVelWeight(int i, real_t _weight){
	velWeight = _weight;
}

void IKJointHandle::SetAccWeight(int i, real_t _weight){
	accWeight = _weight;
}

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
		pos_con[i]->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos[i] && i < joint->ndof);
		vel_con[i]->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel[i] && i < joint->ndof);
		acc_con[i]->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc[i] && i < joint->ndof);

		pos_con[i]->weight = posWeight;
		vel_con[i]->weight = velWeight;
		acc_con[i]->weight = accWeight;
	}
}

void IKJointHandle::Finish(){
}

void IKJointHandle::Update(){
}

void IKJointHandle::Draw(GRRenderIf* render){
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void IKComHandleBase::AddBody(IKBody* _body){
	vector<BodyInfo>::iterator it;
	for(it = bodies.begin(); it != bodies.end(); it++){
		if(it->body == _body)
			break;
	}
	if(it == bodies.end()){
		bodies.push_back(BodyInfo());
		bodies.back().body = _body;
	}
	solver->ready = false;
}

void IKComHandleBase::DeleteBody(IKBody* _body){
	for(vector<BodyInfo>::iterator it = bodies.begin(); it != bodies.end(); ){
		if(it->body == _body)
			 it = bodies.erase(it);
		else it++;
	}
	solver->ready = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::PosCon::PosCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddC3Link(jnt->q_var[n]);
		}
	}
	AddSLink (handle->root->pos_var);
	AddX3Link(handle->root->ori_var);
}
void IKComHandle::PosCon::CalcCoef(){
	uint idx = 0;
	for(uint i = 0; i < handle->joints.size(); i++){
		IKComHandle::JointInfo& jntInfo = handle->joints[i];
		IKJoint* jnt = jntInfo.joint;

		for(int n = 0; n < jnt->ndof; n++){
			vec3_t coef;

			for(uint j = 0; j < jntInfo.bodies.size(); j++){
				IKComHandle::BodyInfo* bodyInfo = jntInfo.bodies[j];
				IKBody* body  = bodyInfo->body;
				real_t  mnorm = bodyInfo->massNorm;

				coef += -1.0 * mnorm * (jnt->Jv_abs[n] + jnt->Jw_abs[n] % (body->centerPosAbs - jnt->sockPosAbs));
			}

			((C3Link*)links[idx++])->SetCoef(coef);
		}
	}

	((SLink* )links[idx++])->SetCoef(-1.0);
	((X3Link*)links[idx++])->SetCoef(handle->comPosAbs - handle->root->pos_var->val);

}
void IKComHandle::PosCon::CalcDeviation(){
	y = handle->desPos - handle->comPosAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::VelCon::VelCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddC3Link(jnt->qd_var[n]);
		}
	}
	AddSLink (handle->root->vel_var   );
	AddX3Link(handle->root->angvel_var);
}

void IKComHandle::VelCon::CalcCoef(){
	uint idx = 0;
	for(uint i = 0; i < handle->joints.size(); i++){
		IKComHandle::JointInfo& jntInfo = handle->joints[i];
		IKJoint* jnt = jntInfo.joint;

		for(int n = 0; n < jnt->ndof; n++){
			vec3_t coef;

			for(uint j = 0; j < jntInfo.bodies.size(); j++){
				IKComHandle::BodyInfo* bodyInfo = jntInfo.bodies[j];
				IKBody* body  = bodyInfo->body;
				real_t  mnorm = bodyInfo->massNorm;

				coef += -1.0 * mnorm * (jnt->Jv_abs[n] + jnt->Jw_abs[n] % (body->centerPosAbs - jnt->sockPosAbs));
			}

			((C3Link*)links[idx++])->SetCoef(coef);
		}
	}

	((SLink* )links[idx++])->SetCoef(-1.0);
	((X3Link*)links[idx++])->SetCoef(handle->comPosAbs - handle->root->pos_var->val);
}

void IKComHandle::VelCon::CalcDeviation(){
	vec3_t ep = handle->desPos - handle->comPosAbs;
	real_t w  = handle->solver->corrRate;
	real_t dt = handle->solver->dt;
	y = (w*(ep/dt) + (1.0-w)*handle->desVel) - handle->comVelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::AccCon::AccCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddC3Link(jnt->qdd_var[n]);
		}
	}
	AddSLink (handle->root->acc_var   );
	AddX3Link(handle->root->angacc_var);
}

void IKComHandle::AccCon::CalcCoef(){
	uint idx = 0;
	for(uint i = 0; i < handle->joints.size(); i++){
		IKComHandle::JointInfo& jntInfo = handle->joints[i];
		IKJoint* jnt = jntInfo.joint;

		for(int n = 0; n < jnt->ndof; n++){
			vec3_t coef;

			for(uint j = 0; j < jntInfo.bodies.size(); j++){
				IKComHandle::BodyInfo* bodyInfo = jntInfo.bodies[j];
				IKBody* body  = bodyInfo->body;
				real_t  mnorm = bodyInfo->massNorm;

				coef += -1.0 * mnorm * (jnt->Jv_abs[n] + jnt->Jw_abs[n] % (body->centerPosAbs - jnt->sockPosAbs));
			}

			((C3Link*)links[idx++])->SetCoef(coef);
		}
	}

	((SLink* )links[idx++])->SetCoef(-1.0);
	((X3Link*)links[idx++])->SetCoef(handle->comPosAbs - handle->root->pos_var->val);
}

void IKComHandle::AccCon::CalcDeviation(){
	y = handle->desAcc - handle->comAccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::MomentumCon::MomentumCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), 1.0){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddC3Link(jnt->qd_var[n]);
		}
	}
	AddM3Link(handle->root->angvel_var);
}

void IKComHandle::MomentumCon::CalcCoef(){
	uint idx = 0;
	for(uint i = 0; i < handle->joints.size(); i++){
		IKComHandle::JointInfo& jntInfo = handle->joints[i];
		IKJoint* jnt = jntInfo.joint;

		for(int n = 0; n < jnt->ndof; n++){
			vec3_t coef;

			for(uint j = 0; j < jntInfo.bodies.size(); j++){
				IKComHandle::BodyInfo* bodyInfo = jntInfo.bodies[j];
				IKBody* body  = bodyInfo->body;
			
				coef += body->inertiaAbs*jnt->Jw_abs[n]
					  + body->mass * ( (body->centerPosAbs - handle->comPosAbs) % (jnt->Jv_abs[n] + jnt->Jw_abs[n] % (body->centerPosAbs - jnt->sockPosAbs)) );
			}

			((C3Link*)links[idx++])->SetCoef(-coef);
		}
	}

	mat3_t Isum;
	Isum.clear();
	for(uint i = 0; i < handle->bodies.size(); i++){
		IKBody* body  = handle->bodies[i].body;

		mat3_t rc = mat3_t::Cross(body->centerPosAbs - handle->comPosAbs);
		Isum += (body->inertiaAbs - body->mass * rc*rc);
	}
	((M3Link*)links[idx++])->SetCoef(-Isum);
}

void IKComHandle::MomentumCon::CalcDeviation(){
	y = handle->desMom - handle->momAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::IKComHandle(IKSolver* _solver, const string& _name){
	name   = _name;
	solver = _solver;
	
	totalMass = 0.0;

	enablePos = false;
	enableVel = false;
	enableAcc = false;
	enableMom = false;

	posWeight = 1.0;
	velWeight = 1.0;
	accWeight = 1.0;
	momWeight = 1.0;
}

void IKComHandle::GetCurrentPos     (      vec3_t& _pos){ _pos      = pos;    }
void IKComHandle::GetCurrentVel     (      vec3_t& _vel){ _vel      = vel;    }
void IKComHandle::GetCurrentAcc     (      vec3_t& _acc){ _acc      = acc;    }
void IKComHandle::GetCurrentMomentum(      vec3_t& _mom){ _mom      = mom;    }

void IKComHandle::SetDesiredPos     (const vec3_t& _pos){ desPos    = _pos;   }
void IKComHandle::SetDesiredVel     (const vec3_t& _vel){ desVel    = _vel;   }
void IKComHandle::SetDesiredAcc     (const vec3_t& _acc){ desAcc    = _acc;   }
void IKComHandle::SetDesiredMomentum(const vec3_t& _mom){ desMom    = _mom;   }

void IKComHandle::GetDesiredPos     (      vec3_t& _pos){ _pos      = desPos; }
void IKComHandle::GetDesiredVel     (      vec3_t& _vel){ _vel      = desVel; }
void IKComHandle::GetDesiredAcc     (      vec3_t& _acc){ _acc      = desAcc; }	
void IKComHandle::GetDesiredMomentum(      vec3_t& _mom){ _mom      = desMom; }

void IKComHandle::EnablePos         (      bool on     ){ enablePos = on;     }
void IKComHandle::EnableVel         (      bool on     ){ enableVel = on;     }
void IKComHandle::EnableAcc         (      bool on     ){ enableAcc = on;     }
void IKComHandle::EnableMomentum    (      bool on     ){ enableMom = on;     }

void IKComHandle::SetPosWeight      (real_t weight     ){ posWeight = weight; }
void IKComHandle::SetVelWeight      (real_t weight     ){ velWeight = weight; }
void IKComHandle::SetAccWeight      (real_t weight     ){ accWeight = weight; }
void IKComHandle::SetMomentumWeight (real_t weight     ){ momWeight = weight; }

real_t IKComHandle::GetTotalMass(){ return totalMass; }

void IKComHandle::Init(){
	for(uint i = 0; i < bodies.size(); i++){
		IKBody* body = bodies[i].body;
		if(!body->parBody){
			root = body;
		}
		else{
			for(IKBody* b = body; b->parBody != 0; b = b->parBody){
				uint j;
				for(j = 0; j < joints.size(); j++){
					if(joints[j].joint == b->parJoint)
						break;
				}
				if(j == joints.size()){
					joints.push_back(JointInfo());
					joints.back().joint = b->parJoint;
				}
				joints[j].bodies.push_back(&bodies[i]);
			}
		}
	}

	totalMass = 0.0;

	for(uint i = 0; i < bodies.size(); i++){
		IKBody* body = bodies[i].body;
		totalMass += body->mass;
	}

	const real_t eps = 1.0e-10;
	if(totalMass > eps){
		for(uint i = 0; i < bodies.size(); i++){
			bodies[i].massNorm = bodies[i].body->mass / totalMass;
		}
	}
}

void IKComHandle::AddVar(){

}

void IKComHandle::AddCon(){
	pos_con = new PosCon     (this, name + "_pos");
	vel_con = new VelCon     (this, name + "_vel");
	acc_con = new AccCon     (this, name + "_acc");
	mom_con = new MomentumCon(this, name + "_mom");
}

void IKComHandle::Prepare(){
	pos_con->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos);
	vel_con->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel);
	acc_con->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc);
	mom_con->enabled = (solver->mode == IKSolver::Mode::Vel && enableMom);

	pos_con->weight = posWeight;
	vel_con->weight = velWeight;
	acc_con->weight = accWeight;
	mom_con->weight = momWeight;
}

void IKComHandle::Finish(){
	if(solver->mode == IKSolver::Mode::Pos){
		pos = comPosAbs;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		vel = comVelAbs;
	}
	if(solver->mode == IKSolver::Mode::Acc){
		acc = comAccAbs;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		mom = momAbs;
	}
}

void IKComHandle::Update(){
	if(solver->mode == IKSolver::Mode::Pos){
		comPosAbs = vec3_t();	
		for(uint i = 0; i < bodies.size(); i++){
			comPosAbs += bodies[i].massNorm * bodies[i].body->centerPosAbs;
		}
	}
	if(solver->mode == IKSolver::Mode::Vel){
		comVelAbs = vec3_t();		
		for(uint i = 0; i < bodies.size(); i++){
			comVelAbs += bodies[i].massNorm * bodies[i].body->centerVelAbs;
		}	
	}
	if(solver->mode == IKSolver::Mode::Acc){
		comAccAbs = vec3_t();
		for(uint i = 0; i < bodies.size(); i++){
			comAccAbs += bodies[i].massNorm * bodies[i].body->centerAccAbs;
		}	
	}
	if(solver->mode == IKSolver::Mode::Vel){
		momAbs = vec3_t();
		for(uint i = 0; i < bodies.size(); i++){
			IKBody* body = bodies[i].body;
			momAbs += body->inertiaAbs*body->angvel_var->val + (body->centerPosAbs - comPosAbs) % (body->mass*body->centerVelAbs);
		}
	}
}

void IKComHandle::Draw(GRRenderIf* render){
	Vec3f p;
	
	if(solver->showHandle){
		glColor4fv((float*)solver->handleColor.rgba);
		render->SetPointSize(5.0f);

		p = pos;
		render->DrawPoint(p);

		p = desPos;
		render->DrawPoint(p);
	}
}	

}
