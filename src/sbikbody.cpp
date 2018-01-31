#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbikhandle.h>

#include <GL/glew.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBody::ForceCon::ForceCon(IKBody* b):body(b), Constraint(b->solver, 3, ID(0, 0, 0, ""), 1.0){
	for(IKJointBase* jnt : body->joints){
		AddC3Link(jnt->force_var[0]);
		AddC3Link(jnt->force_var[1]);
		AddC3Link(jnt->force_var[2]);
	}
}
void IKBody::ForceCon::CalcCoef(){
	uint idx = 0;
	for(IKJointBase* jnt : body->joints){
		((C3Link*)links[idx++])->SetCoef( (jnt->sockBody == body ? 1.0 : -1.0) * jnt->axis[0]);
		((C3Link*)links[idx++])->SetCoef( (jnt->sockBody == body ? 1.0 : -1.0) * jnt->axis[1]);
		((C3Link*)links[idx++])->SetCoef( (jnt->sockBody == body ? 1.0 : -1.0) * jnt->axis[2]);
	}
}
void IKBody::ForceCon::CalcDeviation(){
	y = body->force;
	for(IKJointBase* jnt : body->joints){
		y += (jnt->sockBody == body ? 1.0 : -1.0) * jnt->axis[0] * jnt->force_var[0]->val;
		y += (jnt->sockBody == body ? 1.0 : -1.0) * jnt->axis[1] * jnt->force_var[1]->val;
		y += (jnt->sockBody == body ? 1.0 : -1.0) * jnt->axis[2] * jnt->force_var[2]->val;
	}
	for(IKHandle* handle : body->handles){
		y += -1.0 * handle->force;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBody::MomentCon::MomentCon(IKBody* b):body(b), Constraint(b->solver, 3, ID(0, 0, 0, ""), 1.0){
	for(IKJointBase* jnt : body->joints){
		AddC3Link(jnt->force_var [0]);
		AddC3Link(jnt->force_var [1]);
		AddC3Link(jnt->force_var [2]);
		AddC3Link(jnt->moment_var[0]);
		AddC3Link(jnt->moment_var[1]);
		AddC3Link(jnt->moment_var[2]);
	}
}
void IKBody::MomentCon::CalcCoef(){
	uint idx = 0;
	for(IKJointBase* jnt : body->joints){
		real_t sign = (jnt->sockBody == body ? 1.0 : -1.0);
		vec3_t r    = jnt->sockPosAbs - body->centerPosAbs;
		((C3Link*)links[idx++])->SetCoef( sign * (r % jnt->axis[0]) );
		((C3Link*)links[idx++])->SetCoef( sign * (r % jnt->axis[1]) );
		((C3Link*)links[idx++])->SetCoef( sign * (r % jnt->axis[2]) );
		((C3Link*)links[idx++])->SetCoef( sign * jnt->axis[0]);
		((C3Link*)links[idx++])->SetCoef( sign * jnt->axis[1]);
		((C3Link*)links[idx++])->SetCoef( sign * jnt->axis[2]);
	}
}
void IKBody::MomentCon::CalcDeviation(){
	y = body->moment;
	for(IKJointBase* jnt : body->joints){
		real_t sign = (jnt->sockBody == body ? 1.0 : -1.0);
		vec3_t r    = jnt->sockPosAbs - body->centerPosAbs;
		y += sign * (r % jnt->axis[0]) * jnt->force_var[0]->val;
		y += sign * (r % jnt->axis[1]) * jnt->force_var[1]->val;
		y += sign * (r % jnt->axis[2]) * jnt->force_var[2]->val;
		y += sign * jnt->axis[0] * jnt->moment_var[0]->val;
		y += sign * jnt->axis[1] * jnt->moment_var[1]->val;
		y += sign * jnt->axis[2] * jnt->moment_var[2]->val;
	}
	for(IKHandle* handle : body->handles){
		y += -1.0 * ((handle->sockPosAbs - body->centerPosAbs) % handle->force + handle->moment);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBody::IKBody(IKSolver* _solver, const string& _name){
	name     = _name;
	solver   = _solver;
	parBody  = 0;
	parJoint = 0;

	pos_var    = 0;
	ori_var    = 0;
	vel_var    = 0;
	angvel_var = 0;
	acc_var    = 0;
	angacc_var = 0;

	force_con  = 0;
	moment_con = 0;

	mass = 1.0;
}

void IKBody::SetParent(IKBody* par, IKJoint* _joint){
	if(parBody)
		RemoveFromArray(parBody->children, this);
	parBody  = par;
	parJoint = _joint;
	if(parBody)
		parBody->children.push_back(this);
}

IKBody*  IKBody::GetParent(){ return parBody;  }
IKJoint* IKBody::GetJoint (){ return parJoint; }

void   IKBody::SetMass  (real_t m){ mass = m;    }
real_t IKBody::GetMass  (        ){ return mass; }

void   IKBody::SetCenter(const vec3_t& c){ center = c;      }
void   IKBody::GetCenter(      vec3_t& c){ c      = center; }
void   IKBody::SetPose  (const pose_t& p){ pos = p.Pos(); ori = p.Ori(); }
void   IKBody::GetPose  (      pose_t& p){ p.Pos() = pos; p.Ori() = ori; }

void IKBody::Init(){

}

void IKBody::AddVar(){
	pos_var    = new V3Var(solver, ID(0, 0, 0, name + "_pos"   ), 1.0);
	ori_var    = new QVar (solver, ID(0, 0, 0, name + "_ori"   ), 1.0);
	vel_var    = new V3Var(solver, ID(0, 0, 0, name + "_vel"   ), 1.0);
	angvel_var = new V3Var(solver, ID(0, 0, 0, name + "_angvel"), 1.0);
	acc_var    = new V3Var(solver, ID(0, 0, 0, name + "_acc"   ), 1.0);
	angacc_var = new V3Var(solver, ID(0, 0, 0, name + "_angvel"), 1.0);
}

void IKBody::AddCon(){
	force_con  = new ForceCon (this);
	moment_con = new MomentCon(this);
}

void IKBody::Prepare(){
	pos_var   ->locked = !(!parBody && solver->mode == IKSolver::Mode::Pos);
	ori_var   ->locked = !(!parBody && solver->mode == IKSolver::Mode::Pos);
	vel_var   ->locked = !(!parBody && solver->mode == IKSolver::Mode::Vel);
	angvel_var->locked = !(!parBody && solver->mode == IKSolver::Mode::Vel);
	acc_var   ->locked = !(!parBody && solver->mode == IKSolver::Mode::Acc);
	angacc_var->locked = !(!parBody && solver->mode == IKSolver::Mode::Acc);

	if(solver->mode == IKSolver::Mode::Force){
		force_con ->enabled = (parBody != 0);
		moment_con->enabled = (parBody != 0);
	}
	else{
		force_con ->enabled = false;
		moment_con->enabled = false;
	}
}

void IKBody::Finish(){
	if(solver->mode == IKSolver::Mode::Pos){
		pos    = pos_var   ->val;
		ori    = ori_var   ->val;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		vel    = vel_var   ->val;
		angvel = angvel_var->val;
	}
	if(solver->mode == IKSolver::Mode::Acc){
		acc    = acc_var   ->val;
		angacc = angacc_var->val;

		force  = mass * (centerAccAbs - solver->gravity);

		quat_t q  = ori_var->val;
		quat_t qc = q.Conjugated();
		vec3_t a  = angacc_var->val;
		vec3_t al = qc * a;
		vec3_t w  = angvel_var->val;
		vec3_t wl = qc * w;
		vec3_t nl = inertia * al + wl % (inertia * wl);
		moment    = q * nl;
	}
}

void IKBody::Update(){
	if(solver->mode == IKSolver::Mode::Pos){
		if(!parBody){
			centerPosAbs = pos_var->val + ori_var->val * center;
		}
		else{
			parJoint->CalcRelativePose();
			parJoint->CalcJacobian();
		
			parJoint->sockOffsetAbs = parBody->ori_var->val * parJoint->sockPos;
			parJoint->sockPosAbs    = parBody->pos_var->val + parJoint->sockOffsetAbs;
			parJoint->sockOriAbs    = parBody->ori_var->val * parJoint->sockOri;
			parJoint->plugPosAbs    = parJoint->sockPosAbs + parJoint->sockOriAbs * parJoint->relPos;
			parJoint->plugOriAbs    = parJoint->sockOriAbs * parJoint->relOri;
			ori_var->val            = parJoint->plugOriAbs * parJoint->plugOri.Conjugated();
			parJoint->plugOffsetAbs = ori_var->val * parJoint->plugPos;
			pos_var->val            = parJoint->plugPosAbs - parJoint->plugOffsetAbs;
			centerPosAbs            = pos_var->val + ori_var->val * center;
		
			for(int i = 0; i < parJoint->ndof; i++){
				parJoint->Jv_abs[i] = parJoint->sockOriAbs * parJoint->Jv[i];
				parJoint->Jw_abs[i] = parJoint->sockOriAbs * parJoint->Jw[i];
			}
		}
	}

	if(solver->mode == IKSolver::Mode::Vel){
		if(!parBody){
			vec3_t rc = centerPosAbs - pos_var->val;
			centerVelAbs = vel_var->val + angvel_var->val % rc;
		}
		else{
			vec3_t vvp = parBody->vel_var   ->val;
			vec3_t vwp = parBody->angvel_var->val;
			vec3_t rpc = pos_var->val - parBody->pos_var->val;

			vel_var   ->val = vvp + vwp % rpc;
			angvel_var->val = vwp;

			cv = vwp % (vwp % rpc);
			cw = vec3_t();

			for(int i = 0; i < parJoint->ndof; i++){
				vec3_t vvj = parJoint->Jv_abs[i] * parJoint->qd_var[i]->val;
				vec3_t vwj = parJoint->Jw_abs[i] * parJoint->qd_var[i]->val;
				vec3_t rjc = pos_var->val - parJoint->sockPosAbs;

				vel_var   ->val += vvj + vwj % rjc;
				angvel_var->val += vwj;

				cv += 2.0 * (vwp % (vvj + vwj % rjc));
				cw += vwp % vwj;
			}

			vec3_t rc = centerPosAbs - pos_var->val;
			centerVelAbs = vel_var->val + angvel_var->val % rc;
		}
	}

	if(solver->mode == IKSolver::Mode::Acc){
		if(!parBody){
			vec3_t rc = centerPosAbs - pos_var->val;
			centerAccAbs = acc_var->val + angacc_var->val % rc + angvel_var->val % (angvel_var->val % rc);
		}
		else{
			vec3_t avp = parBody->acc_var   ->val;
			vec3_t awp = parBody->angacc_var->val;
			vec3_t rpc = pos_var->val - parBody->pos_var->val;

			acc_var   ->val = avp + awp % rpc + cv;
			angacc_var->val = awp             + cw;

			for(int i = 0; i < parJoint->ndof; i++){
				vec3_t avj = parJoint->Jv_abs[i] * parJoint->qdd_var[i]->val;
				vec3_t awj = parJoint->Jw_abs[i] * parJoint->qdd_var[i]->val;
				vec3_t rjc = pos_var->val - parJoint->sockPosAbs;

				acc_var   ->val += avj + awj % rjc;
				angacc_var->val += awj;
			}

			vec3_t rc = centerPosAbs - pos_var->val;
			centerAccAbs = acc_var->val + angacc_var->val % rc + angvel_var->val % (angvel_var->val % rc);
		}
	}

	if(solver->mode == IKSolver::Mode::Force){
		
	}

	for(uint i = 0; i < children.size(); i++)
		children[i]->Update();
}

void IKBody::Draw(GRRenderIf* render){
	Vec3f p0, p1;
	
	if(solver->showBody){
		glColor4fv((float*)solver->bodyColor.rgb);
		render->SetPointSize(3.0f);
		p0 = pos;
		render->DrawPoint(p0);
	}
	if(solver->showVel){
		glColor4fv((float*)solver->velColor.rgb);
		p1 = p0 + solver->velScale * vel;
		render->DrawLine(p0, p1);
	}
	if(solver->showAngvel){
		glColor4fv((float*)solver->angvelColor.rgb);
		p1 = p0 + solver->angvelScale * angvel;
		render->DrawLine(p0, p1);
	}
	if(solver->showAcc){
		glColor4fv((float*)solver->accColor.rgb);
		p1 = p0 + solver->accScale * acc;
		render->DrawLine(p0, p1);
	}
	if(solver->showAngacc){
		glColor4fv((float*)solver->angaccColor.rgb);
		p1 = p0 + solver->angaccScale * angacc;
		render->DrawLine(p0, p1);
	}

}

}
