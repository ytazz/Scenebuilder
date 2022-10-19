#include <sbiksolver.h>
#include <sbikcomhandle.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

static const real_t pi  = M_PI;
static const real_t inf = numeric_limits<real_t>::max();

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

IKComHandle::PosCon::PosCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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

IKComHandle::VelCon::VelCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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

IKComHandle::AccCon::AccCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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

IKComHandle::MomentumCon::MomentumCon(IKComHandle* h, const string& _name):handle(h), Constraint(h->solver, 3, ID(0, 0, 0, _name), Constraint::Type::Equality, 1.0){
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

	vec3_t one(1.0, 1.0, 1.0);
	posWeight = one;
	velWeight = one;
	accWeight = one;
	momWeight = one;
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

void IKComHandle::SetPosWeight      (vec3_t weight     ){ posWeight = weight; }
void IKComHandle::SetVelWeight      (vec3_t weight     ){ velWeight = weight; }
void IKComHandle::SetAccWeight      (vec3_t weight     ){ accWeight = weight; }
void IKComHandle::SetMomentumWeight (vec3_t weight     ){ momWeight = weight; }

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

		// calculate whole-body inertia around com using parallel axis formula
		comInertiaAbs.clear();
		for(uint i = 0; i < bodies.size(); i++){
			mat3_t rc = mat3_t::Cross(bodies[i].body->centerPosAbs - comPosAbs);
			comInertiaAbs += bodies[i].body->inertiaAbs - bodies[i].body->mass * rc*rc;
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
