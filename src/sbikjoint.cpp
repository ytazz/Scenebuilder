#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbmessage.h>
#include <sbrollpitchyaw.h>

#include <GL/glew.h>

namespace Scenebuilder{;

const real_t pi  = (real_t)M_PI;
const real_t eps = 0.0001;
const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::ConBase::ConBase(IKMate* _mate, const string& _name):mate(_mate), Constraint(_mate->solver, 0, ID(0, 0, 0, _name), 1.0){	
	switch(mate->type){
	case IKMate::Type::PointToPoint: nelem = 3; break;
	case IKMate::Type::PointToLine : nelem = 2; break;
	case IKMate::Type::PointToPlane: nelem = 1; break;
	case IKMate::Type::Distance    : nelem = 1; break;
	}
}

void IKMate::ConBase::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->sockPosAbs - jnt->sockPosAbs));
			switch(mate->type){
			case IKMate::Type::PointToPoint:
				((C3Link*)links[idx++])->SetCoef(-r);
				break;
			case IKMate::Type::PointToLine:
				((C2Link*)links[idx++])->SetCoef(-vec2_t(r[0], r[1]));
				break;
			case IKMate::Type::PointToPlane:
				((SLink*)links[idx++])->SetCoef(-r[2]);
				break;
			case IKMate::Type::Distance:
				((SLink*)links[idx++])->SetCoef(-mate->pos_diff * r);
				break;
			}
		}
	}
	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			vec3_t r = mate->sockOriAbs.Conjugated() * (jnt->Jv_abs[i] + jnt->Jw_abs[i] % (mate->plugPosAbs - jnt->sockPosAbs));
			switch(mate->type){
			case IKMate::Type::PointToPoint:
				((C3Link*)links[idx++])->SetCoef(r);
				break;
			case IKMate::Type::PointToLine:
				((C2Link*)links[idx++])->SetCoef(vec2_t(r[0], r[1]));
				break;
			case IKMate::Type::PointToPlane:
				((SLink*)links[idx++])->SetCoef(r[2]);
				break;
			case IKMate::Type::Distance:
				((SLink*)links[idx++])->SetCoef(mate->pos_diff * r);
				break;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::PosCon::PosCon(IKMate* _mate, const string& _name):ConBase(_mate, _name){
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			if(nelem == 1)
				AddSLink(jnt->q_var[i]);
			if(nelem == 2)
				AddC2Link(jnt->q_var[i]);
			if(nelem == 3)
				AddC3Link(jnt->q_var[i]);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			if(nelem == 1)
				AddSLink(jnt->q_var[i]);
			if(nelem == 2)
				AddC2Link(jnt->q_var[i]);
			if(nelem == 3)
				AddC3Link(jnt->q_var[i]);
		}
	}
}

void IKMate::PosCon::CalcDeviation(){
	switch(mate->type){
	case IKMate::Type::PointToPoint:
		y = mate->pos_diff;
		break;
	case IKMate::Type::PointToLine:
		y[0] = mate->pos_diff.x;
		y[1] = mate->pos_diff.y;
		break;
	case IKMate::Type::PointToPlane:
		if(mate->rangeMin.z < mate->rangeMax.z){
			if(mate->pos_diff.z <= mate->rangeMin.z){
				y[0] = mate->pos_diff.z - mate->rangeMin.z;
				active = true;
			}
			else if(mate->pos_diff.z >= mate->rangeMax.z){
				y[0] = mate->pos_diff.z - mate->rangeMax.z;
				active = true;
			}
			else{
				y[0] = 0.0;
				active = false;
			}
		}
		else{
			y[0] = mate->pos_diff.z;
			active = true;
		}
		break;
	case IKMate::Type::Distance:
		y[0] = mate->pos_diff.square() - mate->distance * mate->distance;
		break;
	}
}

void IKMate::PosCon::Project(real_t& l, uint k){
	if(mate->type == IKMate::Type::PointToPlane){
		if(mate->rangeMin.z < mate->rangeMax.z){
			if(mate->pos_diff.z <= mate->rangeMin.z)
				l = std::max(0.0, l);
			if(mate->pos_diff.z >= mate->rangeMax.z)
				l = std::min(0.0, l);
		}
	}
}		

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::VelCon::VelCon(IKMate* _mate, const string& _name):ConBase(_mate, _name){
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			if(nelem == 1)
				AddSLink(jnt->qd_var[i]);
			if(nelem == 2)
				AddC2Link(jnt->qd_var[i]);
			if(nelem == 3)
				AddC3Link(jnt->qd_var[i]);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			if(nelem == 1)
				AddSLink(jnt->qd_var[i]);
			if(nelem == 2)
				AddC2Link(jnt->qd_var[i]);
			if(nelem == 3)
				AddC3Link(jnt->qd_var[i]);
		}
	}
}

void IKMate::VelCon::CalcDeviation(){
	switch(mate->type){
	case IKMate::Type::PointToPoint:
		y = mate->vel_diff;
		break;
	case IKMate::Type::PointToLine:
		y[0] = mate->vel_diff[0];
		y[1] = mate->vel_diff[1];
		break;
	case IKMate::Type::PointToPlane:
		y[0] = mate->vel_diff[2];
		break;
	case IKMate::Type::Distance:
		y[0] = mate->pos_diff * mate->vel_diff;
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::AccCon::AccCon(IKMate* _mate, const string& _name):ConBase(_mate, _name){
	for(IKBody* b = mate->sockBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			if(nelem == 1)
				AddSLink(jnt->qdd_var[i]);
			if(nelem == 2)
				AddC2Link(jnt->qdd_var[i]);
			if(nelem == 3)
				AddC3Link(jnt->qdd_var[i]);
		}
	}

	for(IKBody* b = mate->plugBody; b != mate->rootBody; b = b->parBody){
		IKJoint* jnt = b->parJoint;
		for(int i = 0; i < jnt->ndof; i++){
			if(nelem == 1)
				AddSLink(jnt->qdd_var[i]);
			if(nelem == 2)
				AddC2Link(jnt->qdd_var[i]);
			if(nelem == 3)
				AddC3Link(jnt->qdd_var[i]);
		}
	}
}

void IKMate::AccCon::CalcDeviation(){
	switch(mate->type){
	case IKMate::Type::PointToPoint:
		y = mate->acc_diff;
		break;
	case IKMate::Type::PointToLine:
		y[0] = mate->acc_diff[0];
		y[1] = mate->acc_diff[1];
		break;
	case IKMate::Type::PointToPlane:
		y[0] = mate->acc_diff[2];
		break;
	case IKMate::Type::Distance:
		y[0] = mate->pos_diff * mate->acc_diff + mate->vel_diff.square();
		break;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

IKJointBase::IKJointBase(IKSolver* _solver, int _type, const string& _name){
	name   = _name;
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
		force_var [i] = new SVar(solver, ID(0, 0, 0, name + "_force" ), 1.0);
		moment_var[i] = new SVar(solver, ID(0, 0, 0, name + "_moment"), 1.0);
	}
}

void IKJointBase::AddCon(){

}

void IKJointBase::Prepare(){
	for(int i = 0; i < 3; i++){
		force_var [i]->val = 0.0;
		moment_var[i]->val = 0.0;

		force_var [i]->locked = true;
		moment_var[i]->locked = true;

		force_var [i]->weight = solver->damping;
		moment_var[i]->weight = solver->damping;
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
		glColor4fv((float*)solver->jointColor.rgba);
		render->SetLineWidth(1.0f);
		p0 = pos;
		p1 = sockBody->pos;
		render->DrawLine(p0, p1);
		p0 = pos;
		p1 = plugBody->pos;
		render->DrawLine(p0, p1);
	}
	if(solver->showForce){
		glColor4fv((float*)solver->forceColor.rgba);
		p0 = pos;
		p1 = p0 + solver->forceScale * force;
		render->DrawLine(p0, p1);
	}
	if(solver->showMoment){
		glColor4fv((float*)solver->momentColor.rgba);
		p0 = pos;
		p1 = p0 + solver->momentScale * moment;
		render->DrawLine(p0, p1);
	}
}	

///////////////////////////////////////////////////////////////////////////////////////////////////

IKMate::IKMate(IKSolver* _solver, int _type, const string& _name):IKJointBase(_solver, _type, _name){

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

	pos_con = new PosCon(this, name + "_pos");
	vel_con = new VelCon(this, name + "_vel");
	acc_con = new AccCon(this, name + "_acc");
}

void IKMate::Prepare(){
	IKJointBase::Prepare();

	if(solver->mode == IKSolver::Mode::Force){
		if(type == Type::PointToPoint){
			force_var[0]->locked = false;
			force_var[1]->locked = false;
			force_var[2]->locked = false;
		}
	}

	pos_con->enabled = (solver->mode == IKSolver::Mode::Pos);
	vel_con->enabled = (solver->mode == IKSolver::Mode::Vel);
	acc_con->enabled = (solver->mode == IKSolver::Mode::Acc);
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

IKJoint::IKJoint(IKSolver* _solver, int _type, const string& _name):IKJointBase(_solver, _type, _name){
	switch(type){
	case Type::Hinge         : ndof = 1; break;
	case Type::Slider        : ndof = 1; break;
	case Type::Universaljoint: ndof = 2; break;
	case Type::Balljoint     : ndof = 3; break;
	case Type::Freejoint     : ndof = 6; break;
	case Type::Fixjoint      : ndof = 0; break;
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
		tau_lock[i] = false;
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
	qd_limit[0][i] = lower;
	qd_limit[1][i] = upper;
}

void IKJoint::Init(){
	IKJointBase::Init();
}

void IKJoint::AddVar(){
	IKJointBase::AddVar();

	for(int i = 0; i < ndof; i++){
		q_var  [i] = new SVar(solver, ID(0, 0, 0, name + "_q"  ), 1.0);
		qd_var [i] = new SVar(solver, ID(0, 0, 0, name + "_qd" ), 1.0);
		qdd_var[i] = new SVar(solver, ID(0, 0, 0, name + "_qdd"), 1.0);

		q_var  [i]->val = q_ini  [i];
		qd_var [i]->val = qd_ini [i];
		qdd_var[i]->val = qdd_ini[i];
	}
}

void IKJoint::AddCon(){
	IKJointBase::AddCon();
}

void IKJoint::Reset(){
	for(int i = 0; i < ndof; i++){
		q  [i] = q_var  [i]->val = q_ini  [i];
		qd [i] = qd_var [i]->val = qd_ini [i];
		qdd[i] = qdd_var[i]->val = qdd_ini[i];
	}

	if(solver->mode == IKSolver::Mode::Force){
		if(type == Type::Hinge){
			moment_var[2]->val = tau_ini[0];
		}
		if(type == Type::Slider){
			force_var[2]->val  = tau_ini[0];
		}
		if(type == Type::Balljoint){
			moment_var[0]->val = tau_ini[0];
			moment_var[1]->val = tau_ini[1];
			moment_var[2]->val = tau_ini[2];
		}
	}
}

void IKJoint::Prepare(){
	IKJointBase::Prepare();

	for(int i = 0; i < ndof; i++){
		q_var  [i]->locked = (q_lock  [i] || !(solver->mode == IKSolver::Mode::Pos));
		qd_var [i]->locked = (qd_lock [i] || !(solver->mode == IKSolver::Mode::Vel));
		qdd_var[i]->locked = (qdd_lock[i] || !(solver->mode == IKSolver::Mode::Acc));

		q_var  [i]->weight = solver->damping;
		qd_var [i]->weight = solver->damping;
		qdd_var[i]->weight = solver->damping;
	}

	if(solver->mode == IKSolver::Mode::Force){
		force_var [0]->locked = false;
		force_var [1]->locked = false;
		force_var [2]->locked = false;
		moment_var[0]->locked = false;
		moment_var[1]->locked = false;
		moment_var[2]->locked = false;
	
		if(type == Type::Hinge){
			moment_var[2]->locked = tau_lock[0];
		}
		if(type == Type::Slider){
			force_var[2]->locked = tau_lock[0]; 
		}
		if(type == Type::Balljoint){
			moment_var[0]->locked = tau_lock[0];
			moment_var[1]->locked = tau_lock[1];
			moment_var[2]->locked = tau_lock[2];
		}
	}
}

void IKJoint::Limit(){
	if(solver->mode == IKSolver::Mode::Pos){
		for(int i = 0; i < ndof; i++){
			q_var[i]->val = std::min(std::max(q_limit [0][i], q_var[i]->val), q_limit [1][i]);
		}
	}
	if(solver->mode == IKSolver::Mode::Vel){
		for(int i = 0; i < ndof; i++){
			qd_var[i]->val = std::min(std::max(qd_limit[0][i], qd_var [i]->val), qd_limit[1][i]);

			// limit velocity so that position limit will be satisfied dt later
			if(solver->dt != 0.0){
				qd_var[i]->val = std::max(qd_var[i]->val, std::min(0.0, (q_limit[0][i] - q_var[i]->val))/solver->dt);
				qd_var[i]->val = std::min(qd_var[i]->val, std::max(0.0, (q_limit[1][i] - q_var[i]->val))/solver->dt);
			}
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
			q[i] = q_var[i]->val;
		}
	}
	if(solver->mode == IKSolver::Mode::Vel){
		for(int i = 0; i < ndof; i++){
			qd[i] = qd_var[i]->val;
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

void IKJoint::Integrate(real_t dt){
	for(int i = 0; i < ndof; i++){
		q_var[i]->val += qd_var[i]->val * dt;
	}
}

void IKJoint::Draw(GRRenderIf* render){
	IKJointBase::Draw(render);

	Vec3f p0, p1;
	
	if(solver->showTorque){
		glColor4fv((float*)solver->momentColor.rgba);
		p0 = pos;
		p1 = p0 + solver->momentScale * (ori * vec3_t(0.0, 0.0, tau[0]));
		render->DrawLine(p0, p1);
	}
}	

///////////////////////////////////////////////////////////////////////////////////////////////////

IKLimit::IKLimit(IKSolver* _solver, int _type, const string& _name):IKJointBase(_solver, _type, _name){

}

void IKLimit::Update(){
	IKJointBase::Update();

	if(solver->mode == IKSolver::Mode::Pos){
		sockOffsetAbs = sockBody->ori_var->val * sockPos;
		sockPosAbs    = sockBody->pos_var->val + sockOffsetAbs;
		sockOriAbs    = sockBody->ori_var->val * sockOri;
		plugOffsetAbs = plugBody->ori_var->val * plugPos;
		plugPosAbs    = plugBody->pos_var->val + plugOffsetAbs;
		plugOriAbs    = plugBody->ori_var->val * plugOri;
	}
}

real_t IKLimit::CalcError(){
	if(solver->mode == IKSolver::Mode::Pos){
		if(type == IKLimit::Type::Conic){
			// define coordinate frame whole z-axis points from socket origin to plug origin
			mat3_t R;
			R.col(2) = plugPosAbs - sockPosAbs;
			R.col(2).unitize();
			R.col(0) = vec3_t(0.0, 1.0, 0.0) % R.col(2);
			R.col(1) = R.col(2) % R.col(0);

			// z-axes of socket and plug in this coordinate frame
			vec3_t zsock = R.trans() * (sockOriAbs * vec3_t(0.0, 0.0, 1.0));
			vec3_t zplug = R.trans() * (plugOriAbs * vec3_t(0.0, 0.0, 1.0));

			// polar coordinates
			real_t theta_sock = atan2(zsock.y, zsock.x);
			real_t phi_sock   = asin (zsock.z);
			real_t theta_plug = atan2(zplug.y, zplug.x);
			real_t phi_plug   = asin (zplug.z);

			// phi must be smaller than angle
			real_t e;
			e = abs(phi_sock) - angle;
			if(e > 0.0)
				return e;
			e = abs(phi_plug) - angle;
			if(e > 0.0)
				return e;

			// projected angles
			real_t angle_sock = acos(cos(angle)/cos(phi_sock));
			real_t angle_plug = acos(cos(angle)/cos(phi_plug));

			// difference of theta must be smaller than the sum of projected angles
			real_t theta_diff = theta_plug - theta_sock;
			if(theta_diff >  pi) theta_diff -= 2.0*pi;
			if(theta_diff < -pi) theta_diff += 2.0*pi;

			e = abs(theta_diff) - (abs(angle_sock) + abs(angle_plug));
			if(e > 0.0)
				return e;
		}
	}

	return 0.0;
}

}
