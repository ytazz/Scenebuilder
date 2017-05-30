#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>

#include <GL/glew.h>

namespace Scenebuilder{;

const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::PosCon::PosCon(IKJoint* jnt, int _dir):joint(jnt), Constraint(jnt->solver, 1){
	dir = _dir;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->q_var[i]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->q_var[i]);
		}
	}
}

void IKJoint::PosCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = joint->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(-r[dir]);
		}
	}
	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = joint->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef( r[dir]);
		}
	}
}
void IKJoint::PosCon::CalcDeviation(){
	vec3_t r = joint->sockOriAbs.Conjugated() * (joint->plugPosAbs - joint->sockPosAbs);
	y[0] = r[dir];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::VelCon::VelCon(IKJoint* jnt, int _dir):joint(jnt), Constraint(jnt->solver, 1){
	dir = _dir;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qd_var[i]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qd_var[i]);
		}
	}
}
void IKJoint::VelCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = joint->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(-r[dir]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = joint->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef( r[dir]);
		}
	}
}
void IKJoint::VelCon::CalcDeviation(){
	vec3_t r = joint->sockOriAbs.Conjugated() * (joint->plugVelAbs - joint->sockVelAbs);
	y[0] = r[dir];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJoint::AccCon::AccCon(IKJoint* jnt, int _dir):joint(jnt), Constraint(jnt->solver, 1){
	dir = _dir;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qdd_var[i]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			AddSLink(jnt->qdd_var[i]);
		}
	}
}
void IKJoint::AccCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = joint->sockBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = joint->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef(-r[dir]);
		}
	}

	for(IKBody* b = joint->plugBody; b != joint->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = joint->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (joint->plugPosAbs - jnt->sockPosAbs));
			((SLink*)links[idx++])->SetCoef( r[dir]);
		}
	}
}
void IKJoint::AccCon::CalcDeviation(){
	vec3_t r = joint->sockOriAbs.Conjugated() * (joint->plugAccAbs - joint->sockAccAbs);
	y[0] = r[dir];
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
	/*for(IKBody* b = sockBody; b != 0; b = b->parBody){
		Node n;
		n.body   = b;
		n.shared = false;
		sockPath.push_back(n);
	}
	for(IKBody* b = plugBody; b != 0; b = b->parBody){
		Node n;
		n.body   = b;
		n.shared = false;

		bool found = false;
		uint i;
		for(i = 0; i < sockPath.size(); i++){
			if(sockPath[i].body == b){
				found = true;
				break;
			}
		}
		if(found){
			n.shared = true;
			sockPath[i].shared = true;
		}

		plugPath.push_back(n);
	}*/
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
	if(plugBody->parBody == sockBody){
		for(int i = 0; i < ndof; i++){
			q_var  [i] = new SVar(solver);
			qd_var [i] = new SVar(solver);
			qdd_var[i] = new SVar(solver);
		}
	}

	for(int i = 0; i < 3; i++){
		force_var [i] = new SVar(solver);
		moment_var[i] = new SVar(solver);
	}
}

void IKJoint::AddCon(){
	if(plugBody->parBody != sockBody){
		for(int i = 0; i < 3; i++){
			pos_con[i] = new PosCon(this, i);
			vel_con[i] = new VelCon(this, i);
			acc_con[i] = new AccCon(this, i);
		}
	}
}

void IKJoint::Prepare(){
	if(plugBody->parBody == sockBody){
		for(int i = 0; i < ndof; i++){
			q_var  [i]->val = q_ini  [i];
			qd_var [i]->val = qd_ini [i];
			qdd_var[i]->val = qdd_ini[i];

			q_var  [i]->locked = (q_lock  [i] || !(solver->mode == IKSolver::Mode::Pos));
			qd_var [i]->locked = (qd_lock [i] || !(solver->mode == IKSolver::Mode::Vel));
			qdd_var[i]->locked = (qdd_lock[i] || !(solver->mode == IKSolver::Mode::Acc));
		}
	}

	if(solver->mode == IKSolver::Mode::Force){
		for(int i = 0; i < 3; i++){
			force_var [i]->val = 0.0;
			moment_var[i]->val = 0.0;

			force_var [i]->locked = false;
			moment_var[i]->locked = false;
		}
	
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
		if(type == Type::LineToLine){
			force_var [2]->val = 0.0;
			moment_var[0]->val = 0.0;
			moment_var[1]->val = 0.0;
			moment_var[2]->val = 0.0;

			force_var [2]->locked = true;
			moment_var[0]->locked = true;
			moment_var[1]->locked = true;
			moment_var[2]->locked = true;
		}
	}
	else{
		for(int i = 0; i < 3; i++){
			force_var [i]->locked = true;
			moment_var[i]->locked = true;
		}
	}

	if(plugBody->parBody != sockBody){
		if(type == Type::LineToLine){
			pos_con[0]->enabled = (solver->mode == IKSolver::Mode::Pos);
			pos_con[1]->enabled = (solver->mode == IKSolver::Mode::Pos);
			pos_con[2]->enabled = false;
			vel_con[0]->enabled = (solver->mode == IKSolver::Mode::Vel);
			vel_con[1]->enabled = (solver->mode == IKSolver::Mode::Vel);
			vel_con[2]->enabled = false;
			acc_con[0]->enabled = (solver->mode == IKSolver::Mode::Acc);
			acc_con[1]->enabled = (solver->mode == IKSolver::Mode::Acc);
			acc_con[2]->enabled = false;
		}
	}
}

void IKJoint::Finish(){
	if(solver->mode == IKSolver::Mode::Pos){
		if(plugBody->parBody == sockBody){
			for(int i = 0; i < ndof; i++){
				q  [i] = /*q_var  [i]->val;*/std::min(std::max(q_limit [0][i], q_var  [i]->val), q_limit [1][i]);
			}
		}
		pos = sockPosAbs;
		ori = sockOriAbs;
	}
	if(solver->mode == IKSolver::Mode::Vel){
		if(plugBody->parBody == sockBody){
			for(int i = 0; i < ndof; i++){
				qd [i] = /*qd_var [i]->val;*/std::min(std::max(qd_limit[0][i], qd_var [i]->val), qd_limit[1][i]);
			}
		}
	}
	if(solver->mode == IKSolver::Mode::Acc){
		if(plugBody->parBody == sockBody){
			for(int i = 0; i < ndof; i++){
				qdd[i] = qdd_var[i]->val;
			}
		}
	}
	if(solver->mode == IKSolver::Mode::Force){
		for(int i = 0; i < 3; i++){
			forceLocal [i] = force_var [i]->val;
			momentLocal[i] = moment_var[i]->val;
		}
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
		force  = ori * forceLocal ;
		moment = ori * momentLocal;
	}
}

void IKJoint::Update(){
	if(solver->mode == IKSolver::Mode::Pos){
		if(type == Type::LineToLine){
			sockOffsetAbs = sockBody->ori_var->val * sockPos;
			sockPosAbs    = sockBody->pos_var->val + sockOffsetAbs;
			sockOriAbs    = sockBody->ori_var->val * sockOri;
			plugOffsetAbs = plugBody->ori_var->val * plugPos;
			plugPosAbs    = plugBody->pos_var->val + plugOffsetAbs;
			plugOriAbs    = plugBody->ori_var->val * plugOri;
		}

		axis[0] = sockOriAbs * vec3_t(1.0, 0.0, 0.0);
		axis[1] = sockOriAbs * vec3_t(0.0, 1.0, 0.0);
		axis[2] = sockOriAbs * vec3_t(0.0, 0.0, 1.0);	
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
	if(solver->showTorque){
		glColor4fv((float*)solver->momentColor.rgb);
		p0 = pos;
		p1 = p0 + solver->momentScale * (ori * vec3_t(0.0, 0.0, tau[0]));
		render->DrawLine(p0, p1);
	}

}	

}
