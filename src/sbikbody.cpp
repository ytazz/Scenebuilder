#include <sbiksolver.h>
#include <sbikbody.h>

#include <GL/glew.h>

namespace Scenebuilder{;

IKBody::IKBody(IKSolver* _ikSolver){
	ikSolver = _ikSolver;
	parent   = 0;
	handled  = false;
	type     = 0;
	ndof     = 0;
}

void IKBody::AddVars(){
	for(uint i = 0; i < ndof; i++){
		pos[i] = new SVar(ikSolver);
		pos[i]->dmax = (revolutive[i] ? ikSolver->dmaxRevolutive : ikSolver->dmaxPrismatic);
	}
}

void IKBody::DeleteVars(){
	for(uint i = 0; i < ndof; i++)
		ikSolver->DeleteVar(pos[i]);
}

void IKBody::AddCons(){
	//for(uint i = 0; i < ndof; i++)
	//	range[i] = new RangeConS(&ik->ikSolver, pos[i]);
}

void IKBody::DeleteCons(){
	//for(uint i = 0; i < ndof; i++)
	//	ik->ikSolver.DeleteCon(range[i]);
}

IKBody* IKBody::GetParent(){
	return parent;
}

void IKBody::SetParent(IKBody* par, int _type){
	DeleteVars();
	DeleteCons();

	if(parent)
		RemoveFromArray(parent->children, this);
	parent = par;
	parent->children.push_back(this);

	type = _type;
	if(type == JointType::Hinge){
		ndof = 1;
		revolutive[0] = true;
	}
	if(type == JointType::Slider){
		ndof = 1;
		revolutive[0] = false;
	}
	if(type == JointType::Balljoint){
		ndof = 3;
		revolutive[0] = revolutive[1] = revolutive[2] = true;
	}
	if(type == JointType::Fixjoint){
		ndof = 0;
	}

	AddVars();
	AddCons();
}

void IKBody::SetSocketPose(const pose_t& p){
	psock = p;
}

void IKBody::SetPlugPose(const pose_t& p){
	pplug = p;
}

void IKBody::GetSocketPose(pose_t& p){
	p = psock;
}

void IKBody::GetPlugPose(pose_t& p){
	p = pplug;
}

void IKBody::SetPose(const pose_t& p){
	pose = p;
}

void IKBody::GetPose(pose_t& p){
	p = pose;
}

real_t IKBody::GetJointPos(uint i){
	return pos[i]->val;
}

void IKBody::SetJointPos(uint i, real_t _pos){
	pos[i]->val = _pos;
}

void IKBody::LockPos(uint i, bool lock){
	pos[i]->Lock(lock);
}

void IKBody::SetPosLimit(uint i, real_t lower, real_t upper){
	range[i]->_min = lower;
	range[i]->_max = upper;
}

void IKBody::MarkHandled(IKBody* ref){
	handled = true;
	if(!parent || this == ref)
		return;
	parent->MarkHandled(ref);
}

void IKBody::CalcJacobian(){
	if(type == JointType::Hinge){
		Jv[0] = vec3_t();
		Jw[0] = vec3_t(0.0, 0.0, 1.0);
	}
	if(type == JointType::Slider){
		Jv[0] = vec3_t(0.0, 0.0, 1.0);
		Jw[0] = vec3_t();
	}
	if(type == JointType::Balljoint){
		Jv[0].clear();
		Jv[1].clear();
		Jv[2].clear();

		real_t yaw   = pos[0]->val;
		real_t pitch = pos[1]->val;

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
	if(type == JointType::Fixjoint){

	}
}

void IKBody::CalcRelativePose(){
	if(type == JointType::Hinge){
		prel.Pos().clear();
		prel.Ori() = quat_t::Rot(pos[0]->val, 'z');
	}
	if(type == JointType::Slider){
		prel.Pos() = vec3_t(0.0, 0.0, pos[0]->val);
		prel.Ori() = quat_t();
	}
	if(type == JointType::Balljoint){
		real_t yaw   = pos[0]->val;
		real_t pitch = pos[1]->val;
		real_t roll  = pos[2]->val;
		
		quat_t q;
		q.w = cos(pitch/2) * cos(roll/2);
		q.x = sin(pitch/2) * cos(yaw - roll/2);
		q.y = sin(pitch/2) * sin(yaw - roll/2);
		q.z = cos(pitch/2) * sin(roll/2);
		
		prel.Pos() = vec3_t();
		prel.Ori() = quat_t();
	}
	if(type == JointType::Fixjoint){
		prel.Pos() = vec3_t();
		prel.Ori() = quat_t();
	}
}

void IKBody::Prepare(){
	if(handled && parent){
		CalcRelativePose();
		CalcJacobian();
		
		pose_t psock_abs = parent->pose * psock;
		pose  = psock_abs * prel * pplug.Inv();
		pivot = psock_abs.Pos();

		for(uint i = 0; i < ndof; i++){
			Jv_abs[i] = psock_abs.Ori() * Jv[i];
			Jw_abs[i] = psock_abs.Ori() * Jw[i];
		}

	}

	for(uint i = 0; i < children.size(); i++)
		children[i]->Prepare();
}

void IKBody::CompFK(){
	if(parent){
		CalcRelativePose();
		pose = parent->pose * psock * prel * pplug.Inv();
	}
	for(uint i = 0; i < children.size(); i++)
		children[i]->CompFK();
}

void IKBody::Draw(GRRenderIf* render){
	glColor4fv((float*)ikSolver->bodyColor);
	render->SetPointSize(3.0f);

	Vec3f p = pose.Pos();
	render->DrawPoint(p);

	if(parent){
		Vec3f p0, p1;
		render->SetLineWidth(1.0f);
		p0 = parent->pose.Pos();
		p1 = parent->pose * psock.Pos();
		render->DrawLine(p0, p1);
		p0 = pose.Pos();
		p1 = pose * pplug.Pos();
		render->DrawLine(p0, p1);
	}
}

}
