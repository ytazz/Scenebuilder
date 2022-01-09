#include <sbiksolver.h>
#include <sbikbodyhandle.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

static const real_t pi  = M_PI;
static const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBodyHandle::PosCon::PosCon(IKBodyHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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
void IKBodyHandle::PosCon::CalcCoef(){
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
void IKBodyHandle::PosCon::CalcDeviation(){
	y = handle->desPos - handle->sockPosAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBodyHandle::OriCon::OriCon(IKBodyHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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
void IKBodyHandle::OriCon::CalcCoef(){
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
void IKBodyHandle::OriCon::CalcDeviation(){
	quat_t qerror = handle->sockOriAbs.Conjugated() * handle->desOri;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;

	y = handle->sockOriAbs * (theta * axis);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBodyHandle::VelCon::VelCon(IKBodyHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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
void IKBodyHandle::VelCon::CalcCoef(){
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
void IKBodyHandle::VelCon::CalcDeviation(){
	vec3_t ep = handle->desPos - handle->sockPosAbs;
	real_t w  = handle->solver->corrRate;
	real_t dt = handle->solver->dt;
	y = (w*(ep/dt) + (1.0-w)*handle->desVel) - handle->sockVelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBodyHandle::AngvelCon::AngvelCon(IKBodyHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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
void IKBodyHandle::AngvelCon::CalcCoef(){
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
void IKBodyHandle::AngvelCon::CalcDeviation(){
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

IKBodyHandle::AccCon::AccCon(IKBodyHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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
void IKBodyHandle::AccCon::CalcCoef(){
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
void IKBodyHandle::AccCon::CalcDeviation(){
	y = handle->desAcc - handle->sockAccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBodyHandle::AngaccCon::AngaccCon(IKBodyHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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
void IKBodyHandle::AngaccCon::CalcCoef(){
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
void IKBodyHandle::AngaccCon::CalcDeviation(){
	y = handle->desAngacc - handle->sockAngaccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBodyHandle::IKBodyHandle(IKSolver* _solver, IKBody* _body, const string& _name){
	name   = _name;
	solver = _solver;
	body   = _body;
	
	enablePos    = false;
	enableOri    = false;
	enableVel    = false;
	enableAngvel = false;
	enableAcc    = false;
	enableAngacc = false;

	vec3_t one(1.0, 1.0, 1.0);
	posWeight    = one;
	oriWeight    = one;
	velWeight    = one;
	angvelWeight = one;
	accWeight    = one;
	angaccWeight = one;
}

void IKBodyHandle::SetSocketPose   (const pose_t& p){ sockPos = p.Pos(); sockOri = p.Ori(); }
void IKBodyHandle::GetSocketPose   (      pose_t& p){ p.Pos() = sockPos; p.Ori() = sockOri; }

void IKBodyHandle::GetCurrentPos   (vec3_t& _pos   ){ _pos    = pos;    }
void IKBodyHandle::GetCurrentOri   (quat_t& _ori   ){ _ori    = ori;    }
void IKBodyHandle::GetCurrentVel   (vec3_t& _vel   ){ _vel    = vel;    }
void IKBodyHandle::GetCurrentAngvel(vec3_t& _angvel){ _angvel = angvel; }
void IKBodyHandle::GetCurrentAcc   (vec3_t& _acc   ){ _acc    = acc;    }
void IKBodyHandle::GetCurrentAngacc(vec3_t& _angacc){ _angacc = angacc; }

void IKBodyHandle::SetDesiredPos   (const vec3_t& _pos   ){ desPos       = _pos;      }
void IKBodyHandle::SetDesiredOri   (const quat_t& _ori   ){ desOri       = _ori;      }
void IKBodyHandle::SetDesiredVel   (const vec3_t& _vel   ){ desVel       = _vel;      }
void IKBodyHandle::SetDesiredAngvel(const vec3_t& _angvel){ desAngvel    = _angvel;   }
void IKBodyHandle::SetDesiredAcc   (const vec3_t& _acc   ){ desAcc       = _acc;      }
void IKBodyHandle::SetDesiredAngacc(const vec3_t& _angacc){ desAngacc    = _angacc;   }
void IKBodyHandle::GetDesiredPos   (      vec3_t& _pos   ){ _pos         = desPos;    }
void IKBodyHandle::GetDesiredOri   (      quat_t& _ori   ){ _ori         = desOri;    }
void IKBodyHandle::GetDesiredVel   (      vec3_t& _vel   ){ _vel         = desVel;    }
void IKBodyHandle::GetDesiredAngvel(      vec3_t& _angvel){ _angvel      = desAngvel; }
void IKBodyHandle::GetDesiredAcc   (      vec3_t& _acc   ){ _acc         = desAcc;    }
void IKBodyHandle::GetDesiredAngacc(      vec3_t& _angacc){ _angacc      = desAngacc; }

void IKBodyHandle::EnablePos   (bool on){ enablePos    = on; }
void IKBodyHandle::EnableOri   (bool on){ enableOri    = on; }
void IKBodyHandle::EnableVel   (bool on){ enableVel    = on; }
void IKBodyHandle::EnableAngvel(bool on){ enableAngvel = on; }
void IKBodyHandle::EnableAcc   (bool on){ enableAcc    = on; }
void IKBodyHandle::EnableAngacc(bool on){ enableAngacc = on; }

void IKBodyHandle::SetPosWeight   (vec3_t weight){ posWeight    = weight; }
void IKBodyHandle::SetOriWeight   (vec3_t weight){ oriWeight    = weight; }
void IKBodyHandle::SetVelWeight   (vec3_t weight){ velWeight    = weight; }
void IKBodyHandle::SetAngvelWeight(vec3_t weight){ angvelWeight = weight; }
void IKBodyHandle::SetAccWeight   (vec3_t weight){ accWeight    = weight; }
void IKBodyHandle::SetAngaccWeight(vec3_t weight){ angaccWeight = weight; }

void IKBodyHandle::SetForce (const vec3_t& _force ){ force  = _force ; }
void IKBodyHandle::SetMoment(const vec3_t& _moment){ moment = _moment; }

void IKBodyHandle::Init(){

}

void IKBodyHandle::AddVar(){
}

void IKBodyHandle::AddCon(){
	pos_con    = new PosCon   (this, name + "_pos"   );
	ori_con    = new OriCon   (this, name + "_ori"   );
	vel_con    = new VelCon   (this, name + "_vel"   );
	angvel_con = new AngvelCon(this, name + "_angvel");
	acc_con    = new AccCon   (this, name + "_acc"   );
	angacc_con = new AngaccCon(this, name + "_angacc");
}

void IKBodyHandle::Prepare(){
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

void IKBodyHandle::Finish(){
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

void IKBodyHandle::Update(){
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

void IKBodyHandle::Draw(GRRenderIf* render){
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

}
