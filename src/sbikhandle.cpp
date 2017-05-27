#include <sbiksolver.h>
#include <sbikhandle.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::PosCon::PosCon(IKHandle* h):handle(h), Constraint(h->solver, 3){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddCLink(b->parJoint->q_var[i]);
			}
		}
		else{
			AddSLink(b->pos_var);
			AddXLink(b->ori_var);
		}
	}
}
void IKHandle::PosCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((CLink*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (handle->sockPosAbs - jnt->sockPosAbs)) );
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
			((XLink*)links[idx++])->SetCoef(handle->sockPosAbs - b->pos_var->val);
		}
	}
}
void IKHandle::PosCon::CalcDeviation(){
	y = handle->desPos - handle->sockPosAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::OriCon::OriCon(IKHandle* h):handle(h), Constraint(h->solver, 3){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddCLink(b->parJoint->q_var[i]);
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
				((CLink*)links[idx++])->SetCoef(-1.0 * jnt->Jw_abs[i]);
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
		}
	}
}
void IKHandle::OriCon::CalcDeviation(){
	quat_t qerror = handle->sockOriAbs.Conjugated() * handle->desOri;
	y = handle->sockOriAbs * (qerror.Theta() * qerror.Axis());
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::VelCon::VelCon(IKHandle* h):handle(h), Constraint(h->solver, 3){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < handle->body->parJoint->ndof; i++){
				AddCLink(b->parJoint->qd_var[i]);
			}
		}
		else{
			AddSLink(b->vel_var   );
			AddXLink(b->angvel_var);
		}
	}

}
void IKHandle::VelCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((CLink*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (handle->sockPosAbs - jnt->sockPosAbs) ));
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
			((XLink*)links[idx++])->SetCoef(handle->sockPosAbs - b->pos_var->val);
		}
	}
}
void IKHandle::VelCon::CalcDeviation(){
	y = handle->desVel - handle->sockVelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::AngvelCon::AngvelCon(IKHandle* h):handle(h), Constraint(h->solver, 3){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddCLink(b->parJoint->qd_var[i]);
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
				((CLink*)links[idx++])->SetCoef(-1.0 * jnt->Jw_abs[i]);
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
		}
	}
}
void IKHandle::AngvelCon::CalcDeviation(){
	y = handle->desAngvel - handle->sockAngvelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::AccCon::AccCon(IKHandle* h):handle(h), Constraint(h->solver, 3){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddCLink(b->parJoint->qdd_var[i]);
			}
		}
		else{
			AddSLink(b->acc_var   );
			AddXLink(b->angacc_var);
		}
	}
}
void IKHandle::AccCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			IKJoint* jnt = b->parJoint;
			for(int i = 0; i < jnt->ndof; i++){
				((CLink*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (handle->sockPosAbs - jnt->sockPosAbs) ));
			}
		}
		else{
			((SLink*)links[idx++])->SetCoef(-1.0);
			((XLink*)links[idx++])->SetCoef(handle->sockPosAbs - b->pos_var->val);
		}
	}
}
void IKHandle::AccCon::CalcDeviation(){
	y = handle->desAcc - handle->sockAccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKHandle::AngaccCon::AngaccCon(IKHandle* h):handle(h), Constraint(h->solver, 3){
	for(IKBody* b = handle->body; b != 0; b = b->parBody){
		if(b->parBody){
			for(int i = 0; i < b->parJoint->ndof; i++){
				AddCLink(b->parJoint->qdd_var[i]);
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
				((CLink*)links[idx++])->SetCoef(-1.0 * jnt->Jw_abs[i]);
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

IKHandle::IKHandle(IKSolver* _solver, IKBody* _body){
	solver = _solver;
	body   = _body;
	
	enablePos    = false;
	enableOri    = false;
	enableVel    = false;
	enableAngvel = false;
	enableAcc    = false;
	enableAngacc = false;
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

void IKHandle::SetForce (const vec3_t& _force ){ force  = _force ; }
void IKHandle::SetMoment(const vec3_t& _moment){ moment = _moment; }

void IKHandle::Init(){

}

void IKHandle::AddVar(){
}

void IKHandle::AddCon(){
	pos_con    = new PosCon   (this);
	ori_con    = new OriCon   (this);
	vel_con    = new VelCon   (this);
	angvel_con = new AngvelCon(this);
	acc_con    = new AccCon   (this);
	angacc_con = new AngaccCon(this);
}

void IKHandle::Prepare(){
	pos_con   ->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos   );
	ori_con   ->enabled = (solver->mode == IKSolver::Mode::Pos && enableOri   );
	vel_con   ->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel   );
	angvel_con->enabled = (solver->mode == IKSolver::Mode::Vel && enableAngvel);
	acc_con   ->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc   );
	angacc_con->enabled = (solver->mode == IKSolver::Mode::Acc && enableAngacc);
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
		glColor4fv((float*)solver->handleColor.rgb);
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

IKJointHandle::PosCon::PosCon(IKJointHandle* h, int _idx):handle(h), idx(_idx), Constraint(h->solver, 1){
	AddSLink(handle->joint->q_var[idx]);
}
void IKJointHandle::PosCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::PosCon::CalcDeviation(){
	y[0] = handle->desPos[idx] - handle->joint->q_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::VelCon::VelCon(IKJointHandle* h, int _idx):handle(h), idx(_idx), Constraint(h->solver, 1){
	AddSLink(handle->joint->qd_var[idx]);
}
void IKJointHandle::VelCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::VelCon::CalcDeviation(){
	y[0] = handle->desVel[idx] - handle->joint->qd_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::AccCon::AccCon(IKJointHandle* h, int _idx):handle(h), idx(_idx), Constraint(h->solver, 1){
	AddSLink(handle->joint->qdd_var[idx]);
}
void IKJointHandle::AccCon::CalcCoef(){
	((SLink*)links[0])->SetCoef(-1.0);
}
void IKJointHandle::AccCon::CalcDeviation(){
	y[0] = handle->desAcc[idx] - handle->joint->qdd_var[idx]->val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointHandle::IKJointHandle(IKSolver* _solver, IKJoint* _joint){
	solver = _solver;
	joint  = _joint;
	
	for(int i = 0; i < 3; i++){
		enablePos[i] = false;
		enableVel[i] = false;
		enableAcc[i] = false;
	}
}

void IKJointHandle::SetDesiredPos(int i, real_t _pos){ desPos[i] = _pos; }
void IKJointHandle::SetDesiredAcc(int i, real_t _acc){ desAcc[i] = _acc; }
void IKJointHandle::SetDesiredVel(int i, real_t _vel){ desVel[i] = _vel; }

void IKJointHandle::EnablePos(int i, bool on){ enablePos[i] = on; }
void IKJointHandle::EnableVel(int i, bool on){ enableVel[i] = on; }
void IKJointHandle::EnableAcc(int i, bool on){ enableAcc[i] = on; }

void IKJointHandle::Init(){
}

void IKJointHandle::AddVar(){
}

void IKJointHandle::AddCon(){
	for(int i = 0; i < joint->ndof; i++){
		pos_con[i] = new PosCon(this, i);
		vel_con[i] = new VelCon(this, i);
		acc_con[i] = new AccCon(this, i);
	}
}

void IKJointHandle::Prepare(){
	for(int i = 0; i < joint->ndof; i++){
		pos_con[i]->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos[i] && i < joint->ndof);
		vel_con[i]->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel[i] && i < joint->ndof);
		acc_con[i]->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc[i] && i < joint->ndof);
	}
}

void IKJointHandle::Finish(){
}

void IKJointHandle::Update(){
}

void IKJointHandle::Draw(GRRenderIf* render){
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::PosCon::PosCon(IKComHandle* h):handle(h), Constraint(h->solver, 3){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddCLink(jnt->q_var[n]);
		}
	}
	AddSLink(handle->root->pos_var);
	AddXLink(handle->root->ori_var);
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

			((CLink*)links[idx++])->SetCoef(coef);
		}
	}

	((SLink*)links[idx++])->SetCoef(-1.0);
	((XLink*)links[idx++])->SetCoef(handle->comPosAbs - handle->root->pos_var->val);

}
void IKComHandle::PosCon::CalcDeviation(){
	y = handle->desPos - handle->comPosAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::VelCon::VelCon(IKComHandle* h):handle(h), Constraint(h->solver, 3){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddCLink(jnt->qd_var[n]);
		}
	}
	AddSLink(handle->root->vel_var   );
	AddXLink(handle->root->angvel_var);
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

			((CLink*)links[idx++])->SetCoef(coef);
		}
	}

	((SLink*)links[idx++])->SetCoef(-1.0);
	((XLink*)links[idx++])->SetCoef(handle->comPosAbs - handle->root->pos_var->val);
}

void IKComHandle::VelCon::CalcDeviation(){
	y = handle->desVel - handle->comVelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::AccCon::AccCon(IKComHandle* h):handle(h), Constraint(h->solver, 3){
	for(uint i = 0; i < handle->joints.size(); i++){
		IKJoint* jnt = handle->joints[i].joint;
		for(int n = 0; n < jnt->ndof; n++){
			AddCLink(jnt->qdd_var[n]);
		}
	}
	AddSLink(handle->root->acc_var   );
	AddXLink(handle->root->angacc_var);
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

			((CLink*)links[idx++])->SetCoef(coef);
		}
	}

	((SLink*)links[idx++])->SetCoef(-1.0);
	((XLink*)links[idx++])->SetCoef(handle->comPosAbs - handle->root->pos_var->val);
}

void IKComHandle::AccCon::CalcDeviation(){
	y = handle->desAcc - handle->comAccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKComHandle::IKComHandle(IKSolver* _solver){
	solver = _solver;
	
	totalMass = 0.0;
}

void IKComHandle::AddBody(IKBody* _body){
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

void IKComHandle::DeleteBody(IKBody* _body){
	for(vector<BodyInfo>::iterator it = bodies.begin(); it != bodies.end(); ){
		if(it->body == _body)
			 it = bodies.erase(it);
		else it++;
	}
	solver->ready = false;
}

void IKComHandle::GetCurrentPos(      vec3_t& _pos){ _pos      = pos;    }
void IKComHandle::GetCurrentVel(      vec3_t& _vel){ _vel      = vel;    }
void IKComHandle::GetCurrentAcc(      vec3_t& _acc){ _acc      = acc;    }
void IKComHandle::SetDesiredPos(const vec3_t& _pos){ desPos    = _pos;   }
void IKComHandle::SetDesiredVel(const vec3_t& _vel){ desVel    = _vel;   }
void IKComHandle::SetDesiredAcc(const vec3_t& _acc){ desAcc    = _acc;   }
void IKComHandle::GetDesiredPos(      vec3_t& _pos){ _pos      = desPos; }
void IKComHandle::GetDesiredVel(      vec3_t& _vel){ _vel      = desVel; }
void IKComHandle::GetDesiredAcc(      vec3_t& _acc){ _acc      = desAcc; }	
void IKComHandle::EnablePos    (      bool on     ){ enablePos = on;     }
void IKComHandle::EnableVel    (      bool on     ){ enableVel = on;     }
void IKComHandle::EnableAcc    (      bool on     ){ enableAcc = on;     }

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
	pos_con = new PosCon(this);
	vel_con = new VelCon(this);
	acc_con = new AccCon(this);
}

void IKComHandle::Prepare(){
	pos_con->enabled = (solver->mode == IKSolver::Mode::Pos && enablePos);
	vel_con->enabled = (solver->mode == IKSolver::Mode::Vel && enableVel);
	acc_con->enabled = (solver->mode == IKSolver::Mode::Acc && enableAcc);
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
}

void IKComHandle::Draw(GRRenderIf* render){
	Vec3f p;
	
	if(solver->showHandle){
		glColor4fv((float*)solver->handleColor.rgb);
		render->SetPointSize(5.0f);

		p = pos;
		render->DrawPoint(p);

		p = desPos;
		render->DrawPoint(p);
	}
}	

}
