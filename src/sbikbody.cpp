#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbikhandle.h>

#include <GL/glew.h>

namespace Scenebuilder{;

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBody::ForceCon::ForceCon(IKBody* b):body(b), Constraint(b->solver, 3){
	for(uint i = 0; i < body->joints.size(); i++){
		IKJoint* jnt = body->joints[i];
		AddSLink(jnt->force_var);
	}
	for(uint i = 0; i < body->handles.size(); i++){
		IKHandle* handle = body->handles[i];
		AddSLink(handle->force_var);
	}
}
void IKBody::ForceCon::CalcCoef(){
	uint idx = 0;
	for(uint i = 0; i < body->joints.size(); i++){
		IKJoint* jnt = body->joints[i];
		((SLink*)links[idx++])->SetCoef(jnt->sockBody == body ? 1.0 : -1.0);
	}
	for(uint i = 0; i < body->handles.size(); i++){
		IKHandle* handle = body->handles[i];
		((SLink*)links[idx++])->SetCoef(-1.0);
	}
}
void IKBody::ForceCon::CalcDeviation(){
	y = body->force;
	for(uint i = 0; i < body->joints.size(); i++){
		IKJoint* jnt = body->joints[i];
		y += (jnt->sockBody == body ? 1.0 : -1.0) * jnt->force_var->val;
	}
	for(uint i = 0; i < body->handles.size(); i++){
		IKHandle* handle = body->handles[i];
		y += -1.0 * handle->force_var->val;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBody::MomentCon::MomentCon(IKBody* b):body(b), Constraint(b->solver, 3){
	for(uint i = 0; i < body->joints.size(); i++){
		IKJoint* jnt = body->joints[i];
		AddXLink(jnt->force_var );
		AddSLink(jnt->moment_var);
	}
	for(uint i = 0; i < body->handles.size(); i++){
		IKHandle* handle = body->handles[i];
		AddXLink(handle->force_var );
		AddSLink(handle->moment_var);
	}
}
void IKBody::MomentCon::CalcCoef(){
	uint idx = 0;
	for(uint i = 0; i < body->joints.size(); i++){
		IKJoint* jnt = body->joints[i];
		((XLink*)links[idx++])->SetCoef((jnt->sockBody == body ? 1.0 : -1.0) * (jnt->sockPosAbs - body->centerPosAbs));
		((SLink*)links[idx++])->SetCoef( jnt->sockBody == body ? 1.0 : -1.0);
	}
	for(uint i = 0; i < body->handles.size(); i++){
		IKHandle* handle = body->handles[i];
		((XLink*)links[idx++])->SetCoef(-1.0 * (handle->sockPosAbs - body->centerPosAbs));
		((SLink*)links[idx++])->SetCoef(-1.0);
	}
}
void IKBody::MomentCon::CalcDeviation(){
	y = body->moment;
	for(uint i = 0; i < body->joints.size(); i++){
		IKJoint* jnt = body->joints[i];
		y += (jnt->sockBody == body ? 1.0 : -1.0) * ((jnt->sockPosAbs - body->centerPosAbs) % jnt->force_var->val + jnt->moment_var->val);
	}
	for(uint i = 0; i < body->handles.size(); i++){
		IKHandle* handle = body->handles[i];
		y += -1.0 * ((handle->sockPosAbs - body->centerPosAbs) % handle->force_var->val + handle->moment_var->val);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKBody::IKBody(IKSolver* _solver){
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
	pos_var    = new V3Var(solver);
	ori_var    = new QVar (solver);
	vel_var    = new V3Var(solver);
	angvel_var = new V3Var(solver);
	acc_var    = new V3Var(solver);
	angacc_var = new V3Var(solver);
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

	force_con ->enabled = (solver->mode == IKSolver::Mode::Force);
	moment_con->enabled = (solver->mode == IKSolver::Mode::Force);

	if(solver->mode == IKSolver::Mode::Force){
		force  = mass * (centerAccAbs - solver->gravity); 
		moment = ori_var->val * (inertia * (ori_var->val.Conjugated() * angacc_var->val));

		DSTR << "bf " << force << " bm " << moment << endl;
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
		
			parJoint->sockPosAbs = parBody->pos_var->val + parBody->ori_var->val * parJoint->sockPos;
			parJoint->sockOriAbs = parBody->ori_var->val * parJoint->sockOri;
			parJoint->plugPosAbs = parJoint->sockPosAbs + parJoint->sockOriAbs * parJoint->relPos;
			parJoint->plugOriAbs = parJoint->sockOriAbs * parJoint->relOri;

			ori_var->val = parJoint->plugOriAbs * parJoint->plugOri.Conjugated();
			pos_var->val = parJoint->plugPosAbs - ori_var->val * parJoint->plugPos;
			
			centerPosAbs = pos_var->val + ori_var->val * center;
		
			for(uint i = 0; i < parJoint->ndof; i++){
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

			for(uint i = 0; i < parJoint->ndof; i++){
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

			for(uint i = 0; i < parJoint->ndof; i++){
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
	
	glColor4fv((float*)solver->bodyColor.rgb);

	render->SetPointSize(3.0f);
	p0 = pos;
	render->DrawPoint(p0);

	if(parBody){
		render->SetLineWidth(1.0f);
		p0 = parBody ->pos;
		p1 = parJoint->pos;
		render->DrawLine(p0, p1);
		p0 = pos;
		p1 = parJoint->pos;
		render->DrawLine(p0, p1);
	}

	glColor4fv((float*)solver->velColor.rgb);
	p1 = p0 + solver->velScale * vel;
	render->DrawLine(p0, p1);

	glColor4fv((float*)solver->accColor.rgb);
	p1 = p0 + solver->accScale * acc;
	render->DrawLine(p0, p1);

}

}
