#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbmessage.h>
#include <sbrollpitchyaw.h>

#include <GL/glew.h>

namespace Scenebuilder{;

const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::PosCon::PosCon(IKMate* _mate):mate(_mate), Constraint(_mate->solver, 1, ID(0, 0, 0, ""), 1.0){
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->q_var[i]);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->q_var[i]);
		}
	}
}

void IKMate::PosCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(-mate->pos_diff * r);
		}
	}
	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(mate->pos_diff * r);
		}
	}
}
void IKMate::PosCon::CalcDeviation(){
	y[0] = mate->pos_diff.square() - mate->distance * mate->distance;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::VelCon::VelCon(IKMate* _mate):mate(_mate), Constraint(_mate->solver, 1, ID(0, 0, 0, ""), 1.0){
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qd_var[i]);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qd_var[i]);
		}
	}
}
void IKMate::VelCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(-mate->pos_diff * r);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(mate->pos_diff * r);
		}
	}
}

void IKMate::VelCon::CalcDeviation(){
	y[0] = mate->pos_diff * mate->vel_diff;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::AccCon::AccCon(IKMate* _mate):mate(_mate), Constraint(_mate->solver, 1, ID(0, 0, 0, ""), 1.0){
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qdd_var[i]);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qdd_var[i]);
		}
	}
}

void IKMate::AccCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(-mate->pos_diff * r);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(mate->pos_diff * r);
		}
	}
}

void IKMate::AccCon::CalcDeviation(){
	y[0] = mate->pos_diff * mate->acc_diff + mate->vel_diff.square();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointBase::IKJointBase(IKSolver* _solver, int _type){
	solver = _solver;
	type   = _type;

	sockBody = 0;
	plugBody = 0;
}

void IKJointBase::SetSocketBody(IKBody* _sockBody){
	if(sockBody)
		RemoveFromArray(sockBody->joints, this);
	sockBody = _sockBody;
	sockBody->joints.push_back(this);
}
void IKJointBase::SetPlugBody  (IKBody* _plugBody){
	if(plugBody)
		RemoveFromArray(plugBody->joints, this);
	plugBody = _plugBody;
	plugBody->joints.push_back(this);
}

void IKJointBase::SetSocketPose(const pose_t& p){ sockPos = p.Pos(); sockOri = p.Ori(); }
void IKJointBase::SetPlugPose  (const pose_t& p){ plugPos = p.Pos(); plugOri = p.Ori(); }
void IKJointBase::GetSocketPose(      pose_t& p){ p.Pos() = sockPos; p.Ori() = sockOri; }
void IKJointBase::GetPlugPose  (      pose_t& p){ p.Pos() = plugPos; p.Ori() = plugOri; }

void IKJointBase::Init(){

}

void IKJointBase::AddVar(){
	for(int i = 0; i < 3; i++){
		force_var [i] = new SVar(solver, ID(0, 0, 0, ""), 1.0);
		moment_var[i] = new SVar(solver, ID(0, 0, 0, ""), 1.0);
	}
}

void IKJointBase::AddCon(){

}

void IKJointBase::Prepare(){
	if(solver->mode == IKSolver::Mode::Force){
		for(int i = 0; i < 3; i++){
			force_var [i]->val = 0.0;
			moment_var[i]->val = 0.0;

			force_var [i]->locked = false;
			moment_var[i]->locked = false;
		}
	}
}

void IKJointBase::Update(){
	if(solver->mode == IKSolver::Mode::Vel){
		sockVelAbs = sockBody->vel_var->val + sockBody->angvel_var->val % sockOffsetAbs;
		plugVelAbs = plugBody->vel_var->val + plugBody->angvel_var->val % plugOffsetAbs;
	}
	if(solver->mode == IKSolver::Mode::Acc){
		sockAccAbs = sockBody->acc_var->val + sockBody->angacc_var->val % sockOffsetAbs + sockBody->angvel_var->val % (sockBody->angvel_var->val % sockOffsetAbs);
		plugAccAbs = plugBody->acc_var->val + plugBody->angacc_var->val % plugOffsetAbs + plugBody->angvel_var->val % (plugBody->angvel_var->val % plugOffsetAbs);
	}
}

void IKJointBase::Finish(){
	if(solver->mode == IKSolver::Mode::Pos){
		pos = sockPosAbs;
		ori = sockOriAbs;
	}
	if(solver->mode == IKSolver::Mode::Force){
		for(int i = 0; i < 3; i++){
			forceLocal [i] = force_var [i]->val;
			momentLocal[i] = moment_var[i]->val;
		}
		force  = ori * forceLocal ;
		moment = ori * momentLocal;
	}
}

void IKJointBase::Draw(GRRenderIf* render){
	Vec3f p0, p1;
	
	if(solver->showJoint){
		glColor4fv((float*)solver->jointColor.rgb);
		render->SetLineWidth(1.0f);
		p0 = pos;
		p1 = sockBody->pos;
		render->DrawLine(p0, p1);
		p0 = pos;
		p1 = plugBody->pos;
		render->DrawLine(p0, p1);
	}
	if(solver->showForce){
		glColor4fv((float*)solver->forceColor.rgb);
		p0 = pos;
		p1 = p0 + solver->forceScale * force;
		render->DrawLine(p0, p1);
	}
	if(solver->showMoment){
		glColor4fv((float*)solver->momentColor.rgb);
		p0 = pos;
		p1 = p0 + solver->momentScale * moment;
		render->DrawLine(p0, p1);
	}
}	

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::IKMate(IKSolver* _solver, int _type):IKJointBase(_solver, _type){
	if(type == Type::PointToPoint){
		ndof = 0;
	}
}

void IKMate::Init(){
	rootBody = sockBody;
	bool found = false;
	while(rootBody){
		found = false;
		for(IKBody* b = plugBody; b != 0; b = b->parBody){
			if(b == rootBody){
				found = true;
				break;
			}
		}
		if(found)
			break;
		rootBody = rootBody->parBody;
	}

	if(!found){
		Message::Error("sock and plug must belong to the same tree");
		rootBody = 0;
	}
}

void IKMate::AddVar(){
	IKJointBase::AddVar();
}

void IKMate::AddCon(){
	IKJointBase::AddCon();

	pos_con = new PosCon(this);
	vel_con = new VelCon(this);
	acc_con = new AccCon(this);
}

void IKMate::Prepare(){
	IKJointBase::Prepare();

	if(solver->mode == IKSolver::Mode::Force){
		if(type == Type::PointToPoint){
			moment_var[0]->val = 0.0;
			moment_var[1]->val = 0.0;
			moment_var[2]->val = 0.0;

			moment_var[0]->locked = true;
			moment_var[1]->locked = true;
			moment_var[2]->locked = true;
		}
	}

	if(type == Type::PointToPoint){
		pos_con->enabled = (solver->mode == IKSolver::Mode::Pos);
		vel_con->enabled = (solver->mode == IKSolver::Mode::Vel);
		acc_con->enabled = (solver->mode == IKSolver::Mode::Acc);
	}
}

void IKMate::Update(){
	IKJointBase::Update();

	if(solver->mode == IKSolver::Mode::Pos){
		sockOffsetAbs = sockBody->ori_var->val * sockPos;
		sockPosAbs    = sockBody->pos_var->val + sockOffsetAbs;
		sockOriAbs    = sockBody->ori_var->val * sockOri;
		plugOffsetAbs = plugBody->ori_var->val * plugPos;
		plugPosAbs    = plugBody->pos_var->val + plugOffsetAbs;
		plugOriAbs    = plugBody->ori_var->val * plugOri;
	
		pos_diff = sockOriAbs.Conjugated() * (plugPosAbs - sockPosAbs);
		vel_diff = sockOriAbs.Conjugated() * (plugVelAbs - sockVelAbs);
		acc_diff = sockOriAbs.Conjugated() * (plugAccAbs - sockAccAbs);

		axis[0] = sockOriAbs * vec3_t(1.0, 0.0, 0.0);
		axis[1] = sockOriAbs * vec3_t(0.0, 1.0, 0.0);
		axis[2] = sockOriAbs * vec3_t(0.0, 0.0, 1.0);	
	}
}

void IKMate::Finish(){
	IKJointBase::Finish();
}

void IKMate::Draw(GRRenderIf* render){
	IKJointBase::Draw(render);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::IKJoint(IKSolver* _solver, int _type):IKJointBase(_solver, _type){
	if(type == Type::Hinge){
		ndof = 1;
	}
	if(type == Type::Slider){
		ndof = 1;
	}
	if(type == Type::Universaljoint){
		ndof = 2;
	}
	if(type == Type::Balljoint){
		ndof = 3;
	}
	if(type == Type::Freejoint){
		ndof = 6;
	}
	if(type == Type::Fixjoint){
		ndof = 0;
	}

	for(int i = 0; i < 6; i++){
		q_var  [i] = 0;
		qd_var [i] = 0;
		qdd_var[i] = 0;

		q_ini  [i] = 0.0;
		qd_ini [i] = 0.0;
		qdd_ini[i] = 0.0;

		q_limit [0][i] = -inf;
		q_limit [1][i] =  inf;
		qd_limit[0][i] = -inf;
		qd_limit[1][i] =  inf;

		q_lock  [i] = false;
		qd_lock [i] = false;
		qdd_lock[i] = false;
	}
}

real_t IKJoint::GetPos   (uint i){ return q  [i]; }
real_t IKJoint::GetVel   (uint i){ return qd [i]; }
real_t IKJoint::GetAcc   (uint i){ return qdd[i]; }
real_t IKJoint::GetTorque(uint i){ return tau[i]; }

void IKJoint::LockPos   (uint i, bool lock){ q_lock  [i] = lock; }
void IKJoint::LockVel   (uint i, bool lock){ qd_lock [i] = lock; }
void IKJoint::LockAcc   (uint i, bool lock){ qdd_lock[i] = lock; }
void IKJoint::LockTorque(uint i, bool lock){ tau_lock[i] = lock; }

void IKJoint::SetInitialPos   (uint i, real_t _q  ){ q_ini  [i] = _q  ; }
void IKJoint::SetInitialVel   (uint i, real_t _qd ){ qd_ini [i] = _qd ; }
void IKJoint::SetInitialAcc   (uint i, real_t _qdd){ qdd_ini[i] = _qdd; }
void IKJoint::SetInitialTorque(uint i, real_t _tau){ tau_ini[i] = _tau; }

void IKJoint::SetPosLimit(uint i, real_t lower, real_t upper){
	q_limit[0][i] = lower;
	q_limit[1][i] = upper;
}
void IKJoint::SetVelLimit(uint i, real_t lower, real_t upper){
	qd_limit[i][0] = lower;
	qd_limit[i][1] = upper;
}

void IKJoint::Init(){
	IKJointBase::Init();
}

void IKJoint::AddVar(){
	IKJointBase::AddVar();

	for(int i = 0; i < ndof; i++){
		q_var  [i] = new SVar(solver, ID(0, 0, 0, ""), 1.0);
		qd_var [i] = new SVar(solver, ID(0, 0, 0, ""), 1.0);
		qdd_var[i] = new SVar(solver, ID(0, 0, 0, ""), 1.0);
	}
}

void IKJoint::AddCon(){
	IKJointBase::AddCon();
}

void IKJoint::Prepare(){
	IKJointBase::Prepare();

	for(int i = 0; i < ndof; i++){
		q_var  [i]->val = q_ini  [i];
		qd_var [i]->val = qd_ini [i];
		qdd_var[i]->val = qdd_ini[i];

		q_var  [i]->locked = (q_lock  [i] || !(solver->mode == IKSolver::Mode::Pos));
		qd_var [i]->locked = (qd_lock [i] || !(solver->mode == IKSolver::Mode::Vel));
		qdd_var[i]->locked = (qdd_lock[i] || !(solver->mode == IKSolver::Mode::Acc));
	}
	
	if(solver->mode == IKSolver::Mode::Force){
		if(type == Type::Hinge){
			moment_var[2]->val    = tau_ini [0];
			moment_var[2]->locked = tau_lock[0];
		}
		if(type == Type::Slider){
			force_var[2]->val    = tau_ini [0];
			force_var[2]->locked = tau_lock[0]; 
		}
		if(type == Type::Balljoint){
			moment_var[0]->val    = tau_ini [0];
			moment_var[1]->val    = tau_ini [1];
			moment_var[2]->val    = tau_ini [2];
			moment_var[0]->locked = tau_lock[0];
			moment_var[1]->locked = tau_lock[1];
			moment_var[2]->locked = tau_lock[2];
		}
		if(type == Type::Freejoint){
			force_var [0]->val = 0.0;
			force_var [1]->val = 0.0;
			force_var [2]->val = 0.0;
			moment_var[0]->val = 0.0;
			moment_var[1]->val = 0.0;
			moment_var[2]->val = 0.0;

			force_var [0]->locked = true;
			force_var [1]->locked = true;
			force_var [2]->locked = true;
			moment_var[0]->locked = true;
			moment_var[1]->locked = true;
			moment_var[2]->locked = true;
		}
	}
}

void IKJoint::Update(){
	IKJointBase::Update();
	
	if(solver->mode == IKSolver::Mode::Pos){
		axis[0] = sockOriAbs * vec3_t(1.0, 0.0, 0.0);
		axis[1] = sockOriAbs * vec3_t(0.0, 1.0, 0.0);
		axis[2] = sockOriAbs * vec3_t(0.0, 0.0, 1.0);	
	}
}

void IKJoint::Finish(){
	IKJointBase::Finish();

	if(solver->mode == IKSolver::Mode::Pos){
		for(int i = 0; i < ndof; i++){
			q[i] = std::min(std::max(q_limit [0][i], q_var[i]->val), q_limit [1][i]);
		}
	}
	if(solver->mode == IKSolver::Mode::Vel){
		for(int i = 0; i < ndof; i++){
			qd [i] = std::min(std::max(qd_limit[0][i], qd_var [i]->val), qd_limit[1][i]);
		}
	}
	if(solver->mode == IKSolver::Mode::Acc){
		for(int i = 0; i < ndof; i++){
			qdd[i] = qdd_var[i]->val;
		}
	}
	if(solver->mode == IKSolver::Mode::Force){
		if(type == Type::Hinge){
			tau[0] = momentLocal[2];
		}
		if(type == Type::Slider){
			tau[0] = forceLocal[2];
		}
		if(type == Type::Balljoint){
			tau[0] = momentLocal[0];
			tau[1] = momentLocal[1];
			tau[2] = momentLocal[2];
		}
	}
}

void IKJoint::CalcJacobian(){
	for(int i = 0; i < 6; i++){
		Jv[i].clear();
		Jw[i].clear();
	}
	if(type == Type::Hinge){
		Jw[0] = vec3_t(0.0, 0.0, 1.0);
	}
	if(type == Type::Slider){
		Jv[0] = vec3_t(0.0, 0.0, 1.0);
	}
	if(type == Type::Universaljoint){
		real_t pitch = q_var[1]->val;
		
		// 行列の場合と行・列のインデックス順が逆なので注意
		Jw[0][0] =  cos(pitch);
		Jw[0][1] =  0.0;
		Jw[0][2] = -sin(pitch);
		Jw[1][0] =  0.0;
		Jw[1][1] =  1.0;
		Jw[1][2] =  0.0;
	}
	if(type == Type::Balljoint){
		real_t pitch = q_var[1]->val;
		real_t yaw   = q_var[2]->val;
		
		// 行列の場合と行・列のインデックス順が逆なので注意
		Jw[0][0] =  cos(yaw)*cos(pitch);
		Jw[0][1] =  sin(yaw)*cos(pitch);
		Jw[0][2] = -sin(pitch);
		Jw[1][0] = -sin(yaw);
		Jw[1][1] =  cos(yaw);
		Jw[1][2] =  0.0;
		Jw[2][0] =  0.0;
		Jw[2][1] =  0.0;
		Jw[2][2] =  1.0;
	}
	if(type == Type::Freejoint){
		Jv[0][0] = 1.0;
		Jv[1][1] = 1.0;
		Jv[2][2] = 1.0;

		real_t pitch = q_var[4]->val;
		real_t yaw   = q_var[5]->val;
		
		Jw[3][0] =  cos(yaw)*cos(pitch);
		Jw[3][1] =  sin(yaw)*cos(pitch);
		Jw[3][2] = -sin(pitch);
		Jw[4][0] = -sin(yaw);
		Jw[4][1] =  cos(yaw);
		Jw[4][2] =  0.0;
		Jw[5][0] =  0.0;
		Jw[5][1] =  0.0;
		Jw[5][2] =  1.0;
	}
	if(type == Type::Fixjoint){

	}
}

void IKJoint::CalcRelativePose(){
	relPos = vec3_t();
	relOri = quat_t();

	if(type == Type::Hinge){
		relOri = quat_t::Rot(q_var[0]->val, 'z');
	}
	if(type == Type::Slider){
		relPos = vec3_t(0.0, 0.0, q_var[0]->val);
	}
	if(type == Type::Universaljoint){
		relOri = FromRollPitchYaw(vec3_t(q_var[0]->val, q_var[1]->val, 0.0));
	}
	if(type == Type::Balljoint){
		relOri = FromRollPitchYaw(vec3_t(q_var[0]->val, q_var[1]->val, q_var[2]->val));
	}
	if(type == Type::Freejoint){
		relPos[0] = q_var[0]->val;
		relPos[1] = q_var[1]->val;
		relPos[2] = q_var[2]->val;
		relOri = FromRollPitchYaw(vec3_t(q_var[3]->val, q_var[4]->val, q_var[5]->val));
	}
}

void IKJoint::Draw(GRRenderIf* render){
	IKJointBase::Draw(render);

	Vec3f p0, p1;
	
	if(solver->showTorque){
		glColor4fv((float*)solver->momentColor.rgb);
		p0 = pos;
		p1 = p0 + solver->momentScale * (ori * vec3_t(0.0, 0.0, tau[0]));
		render->DrawLine(p0, p1);
	}
}	

}
