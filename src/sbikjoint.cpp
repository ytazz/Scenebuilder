#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::PosCon::PosCon(IKJoint* jnt):joint(jnt), Constraint(jnt->solver, 3){
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			AddCLink(jnt->q_var[i]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			AddCLink(jnt->q_var[i]);
		}
	}
}
void IKJoint::PosCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			((CLink*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->sockPosAbs - jnt->sockPosAbs)) );
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			((CLink*)links[idx++])->SetCoef(+1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs)) );
		}
	}
}
void IKJoint::PosCon::CalcDeviation(){
	y = joint->plugPosAbs - joint->sockPosAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::VelCon::VelCon(IKJoint* jnt):joint(jnt), Constraint(jnt->solver, 3){
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			AddCLink(jnt->qd_var[i]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			AddCLink(jnt->qd_var[i]);
		}
	}
}
void IKJoint::VelCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			((CLink*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->sockPosAbs - jnt->sockPosAbs)) );
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			((CLink*)links[idx++])->SetCoef(+1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs)) );
		}
	}
}
void IKJoint::VelCon::CalcDeviation(){
	y = joint->plugVelAbs - joint->sockVelAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::AccCon::AccCon(IKJoint* jnt):joint(jnt), Constraint(jnt->solver, 3){
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			AddCLink(jnt->qdd_var[i]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			AddCLink(jnt->qdd_var[i]);
		}
	}
}
void IKJoint::AccCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			((CLink*)links[idx++])->SetCoef(-1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->sockPosAbs - jnt->sockPosAbs)) );
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(uint i = 0; i < jnt->ndof; i++){
			((CLink*)links[idx++])->SetCoef(+1.0 * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs)) );
		}
	}
}
void IKJoint::AccCon::CalcDeviation(){
	y = joint->plugAccAbs - joint->sockAccAbs;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::IKJoint(IKSolver* _solver, int _type){
	solver = _solver;
	type   = _type;

	sockBody = 0;
	plugBody = 0;
	
	if(type == Type::Hinge){
		ndof = 1;
		revolutive[0] = true;
	}
	if(type == Type::Slider){
		ndof = 1;
		revolutive[0] = false;
	}
	if(type == Type::Balljoint){
		ndof = 3;
		revolutive[0] = revolutive[1] = revolutive[2] = true;
	}
	if(type == Type::Fixjoint){
		ndof = 0;
	}
	if(type == Type::LineToLine){
		ndof = 0;
	}

	for(int i = 0; i < 3; i++){
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

	force_var  = 0;
	moment_var = 0;
}

void IKJoint::SetSocketBody(IKBody* _sockBody){
	if(sockBody)
		RemoveFromArray(sockBody->joints, this);
	sockBody = _sockBody;
	sockBody->joints.push_back(this);
}
void IKJoint::SetPlugBody  (IKBody* _plugBody){
	if(plugBody)
		RemoveFromArray(plugBody->joints, this);
	plugBody = _plugBody;
	plugBody->joints.push_back(this);
}

void IKJoint::SetSocketPose(const pose_t& p){ sockPos = p.Pos(); sockOri = p.Ori(); }
void IKJoint::SetPlugPose  (const pose_t& p){ plugPos = p.Pos(); plugOri = p.Ori(); }
void IKJoint::GetSocketPose(      pose_t& p){ p.Pos() = sockPos; p.Ori() = sockOri; }
void IKJoint::GetPlugPose  (      pose_t& p){ p.Pos() = plugPos; p.Ori() = plugOri; }

real_t IKJoint::GetPos   (uint i){ return q  [i]; }
real_t IKJoint::GetVel   (uint i){ return qd [i]; }
real_t IKJoint::GetAcc   (uint i){ return qdd[i]; }
real_t IKJoint::GetTorque(uint i){ return 0.0;    }

void IKJoint::LockPos(uint i, bool lock){ q_lock  [i] = lock; }
void IKJoint::LockVel(uint i, bool lock){ qd_lock [i] = lock; }
void IKJoint::LockAcc(uint i, bool lock){ qdd_lock[i] = lock; }

void IKJoint::SetInitialPos(uint i, real_t _q  ){ q_ini  [i] = _q  ; }
void IKJoint::SetInitialVel(uint i, real_t _qd ){ qd_ini [i] = _qd ; }
void IKJoint::SetInitialAcc(uint i, real_t _qdd){ qdd_ini[i] = _qdd; }

void IKJoint::SetPosLimit(uint i, real_t lower, real_t upper){
	q_limit[0][i] = lower;
	q_limit[1][i] = upper;
}
void IKJoint::SetVelLimit(uint i, real_t lower, real_t upper){
	qd_limit[i][0] = lower;
	qd_limit[i][1] = upper;
}

void IKJoint::Init(){
	rootBody = sockBody;
	while(true){
		bool found = false;
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
}

void IKJoint::AddVar(){
	for(uint i = 0; i < ndof; i++){
		q_var  [i] = new SVar(solver);
		qd_var [i] = new SVar(solver);
		qdd_var[i] = new SVar(solver);
	}

	force_var  = new V3Var(solver);
	moment_var = new V3Var(solver);
}

void IKJoint::AddCon(){
	pos_con = new PosCon(this);
	vel_con = new VelCon(this);
	acc_con = new AccCon(this);
}

void IKJoint::Prepare(){
	for(uint i = 0; i < ndof; i++){
		q_var  [i]->val = q_ini  [i];
		qd_var [i]->val = qd_ini [i];
		qdd_var[i]->val = qdd_ini[i];

		q_var  [i]->locked = (q_lock  [i] || !(plugBody->parBody == sockBody && solver->mode == IKSolver::Mode::Pos));
		qd_var [i]->locked = (qd_lock [i] || !(plugBody->parBody == sockBody && solver->mode == IKSolver::Mode::Vel));
		qdd_var[i]->locked = (qdd_lock[i] || !(plugBody->parBody == sockBody && solver->mode == IKSolver::Mode::Acc));
	}
	
	force_var ->val.clear();
	moment_var->val.clear();

	force_var ->locked = !(solver->mode == IKSolver::Mode::Force);
	moment_var->locked = !(solver->mode == IKSolver::Mode::Force);

	pos_con->enabled = (plugBody->parBody != sockBody && solver->mode == IKSolver::Mode::Pos);
	vel_con->enabled = (plugBody->parBody != sockBody && solver->mode == IKSolver::Mode::Vel);
	acc_con->enabled = (plugBody->parBody != sockBody && solver->mode == IKSolver::Mode::Acc);
}

void IKJoint::Finish(){
	if(solver->mode == IKSolver::Mode::Pos){
		for(uint i = 0; i < ndof; i++){
			q  [i] = q_var  [i]->val;//std::min(std::max(q_limit [0][i], q  [i]->val), q_limit [1][i]);
		}
		pos = sockPosAbs;
		ori = sockOriAbs;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		for(uint i = 0; i < ndof; i++){
			qd [i] = qd_var [i]->val;//std::min(std::max(qd_limit[0][i], qd [i]->val), qd_limit[1][i]);
		}
	
	}
	if(solver->mode == IKSolver::Mode::Acc){
		for(uint i = 0; i < ndof; i++){
			qdd[i] = qdd_var[i]->val;
		}
	}
	if(solver->mode == IKSolver::Mode::Force){
		force  = force_var ->val;
		moment = moment_var->val;
		DSTR << "f " << force << " m " << moment << endl;
	}
}

void IKJoint::Update(){
	if(solver->mode == IKSolver::Mode::Pos){
		sockOffsetAbs = sockBody->ori_var->val * sockPos;
		sockPosAbs    = sockBody->pos_var->val + sockOffsetAbs;
		sockOriAbs    = sockBody->ori_var->val * sockOri;
		plugOffsetAbs = plugBody->ori_var->val * plugPos;
		plugPosAbs    = plugBody->pos_var->val + plugOffsetAbs;
		plugOriAbs    = plugBody->ori_var->val * plugOri;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		sockVelAbs = sockBody->vel_var->val + sockBody->angvel_var->val % sockOffsetAbs;
		plugVelAbs = plugBody->vel_var->val + plugBody->angvel_var->val % plugOffsetAbs;
	}
	if(solver->mode == IKSolver::Mode::Acc){
		sockAccAbs = sockBody->acc_var->val + sockBody->angacc_var->val % sockOffsetAbs + sockBody->angvel_var->val % (sockBody->angvel_var->val % sockOffsetAbs);
		plugAccAbs = plugBody->acc_var->val + plugBody->angacc_var->val % plugOffsetAbs + plugBody->angvel_var->val % (plugBody->angvel_var->val % plugOffsetAbs);
	}
}

void IKJoint::CalcJacobian(){
	if(type == Type::Hinge){
		Jv[0] = vec3_t();
		Jw[0] = vec3_t(0.0, 0.0, 1.0);
	}
	if(type == Type::Slider){
		Jv[0] = vec3_t(0.0, 0.0, 1.0);
		Jw[0] = vec3_t();
	}
	if(type == Type::Balljoint){
		Jv[0].clear();
		Jv[1].clear();
		Jv[2].clear();

		real_t yaw   = q_var[0]->val;
		real_t pitch = q_var[1]->val;

		// 行列の場合と行・列のインデックス順が逆なので注意
		Jw[0][0] = -sin(yaw) * sin(pitch);
		Jw[0][1] =  cos(yaw) * sin(pitch);
		Jw[0][2] =  1.0 - cos(pitch);
		Jw[1][0] =  cos(yaw);
		Jw[1][1] =  sin(yaw);
		Jw[1][2] =  0.0;
		Jw[2][0] =  sin(yaw) * sin(pitch);
		Jw[2][1] = -cos(yaw) * sin(pitch);
		Jw[2][2] =  cos(pitch);
	}
	if(type == Type::Fixjoint){

	}
}

void IKJoint::CalcRelativePose(){
	if(type == Type::Hinge){
		relPos.clear();
		relOri = quat_t::Rot(q_var[0]->val, 'z');
	}
	if(type == Type::Slider){
		relPos = vec3_t(0.0, 0.0, q_var[0]->val);
		relOri = quat_t();
	}
	if(type == Type::Balljoint){
		real_t yaw   = q_var[0]->val;
		real_t pitch = q_var[1]->val;
		real_t roll  = q_var[2]->val;
		
		quat_t q;
		q.w = cos(pitch/2) * cos(roll/2);
		q.x = sin(pitch/2) * cos(yaw - roll/2);
		q.y = sin(pitch/2) * sin(yaw - roll/2);
		q.z = cos(pitch/2) * sin(roll/2);
		
		relPos = vec3_t();
		relOri = quat_t();
	}
	if(type == Type::Fixjoint){
		relPos = vec3_t();
		relOri = quat_t();
	}
}

void IKJoint::Draw(GRRenderIf* render){
	glColor4fv((float*)solver->forceColor.rgb);
	
	Vec3f p0, p1;
	
	p0 = pos;
	p1 = p0 + solver->forceScale * force;
	render->DrawLine(p0, p1);

}	

}
