#include <sbiksolver.h>
#include <sbikhandle.h>
#include <sbikbody.h>

#include <GL/glew.h>

namespace Scenebuilder{;

IKHandle::IKHandle(IKSolver* _ikSolver, IKBody* _endBody, IKBody* _refBody){
	ikSolver = _ikSolver;
	endBody  = _endBody;
	refBody  = _refBody;

	enablePos = true;
	enableOri = false;

	AddCons();
}

void IKHandle::AddCons(){
	con_fix_pos = new FixPosCon(ikSolver, this);
	con_fix_ori = new FixOriCon(ikSolver, this);
}

void IKHandle::DeleteCons(){
	ikSolver->DeleteCon(con_fix_pos);
	ikSolver->DeleteCon(con_fix_ori);
}

void IKHandle::SetEndPose(const pose_t& p){
	endPose = p;
}
void IKHandle::SetRefPose(const pose_t& p){
	refPose = p;
}
void IKHandle::GetEndPose(pose_t& p){
	p = endPose;
}
void IKHandle::GetRefPose(pose_t& p){
	p = refPose;
}
void IKHandle::SetDesiredPose(const pose_t& p){
	desPose = p;
}
void IKHandle::GetDesiredPose(pose_t& p){
	p = desPose;
}
void IKHandle::GetCurrentPose(pose_t& p){
	p = curPose;
}
void IKHandle::EnablePos(bool on){
	enablePos = on;
}
void IKHandle::EnableOri(bool on){
	enableOri = on;
}

void IKHandle::Prepare(){
	if(!endBody || !refBody){
		con_fix_pos->enabled = false;
		con_fix_ori->enabled = false;
		return;
	}

	endPoseAbs = endBody->pose * endPose;
	refPoseAbs = refBody->pose * refPose;
	
	con_fix_pos->enabled = enablePos;
	con_fix_ori->enabled = enableOri;
}

void IKHandle::CompFK(){
	endPoseAbs = endBody->pose * endPose;
	refPoseAbs = refBody->pose * refPose;
	
	curPose = refPoseAbs.Inv() * endPoseAbs;
}	

void IKHandle::Draw(GRRenderIf* render){
	glColor4fv((float*)ikSolver->handleColor);

	Vec3f p0, p1;
	render->SetLineWidth(1.0f);
	p0 = endBody->pose.Pos();
	p1 = endPoseAbs.Pos();
	render->DrawLine(p0, p1);
	
	render->SetPointSize(3.0f);
	Vec3f p = refBody->pose * desPose.Pos();
	render->DrawPoint(p);
}

//-------------------------------------------------------------------------------------------------
// constructors

FixPosCon::FixPosCon(IKSolver* _solver, IKHandle* h):Constraint(_solver, 3){
	handle = h;
	for(IKBody* b = handle->endBody; b != handle->refBody; b = b->parent){
		for(uint i = 0; i < b->ndof; i++)
			AddCLink(b->pos[i]);
	}
}

FixOriCon::FixOriCon(IKSolver* _solver, IKHandle* h):Constraint(_solver, 3){
	handle = h;
	for(IKBody* b = handle->endBody; b != handle->refBody; b = b->parent){
		for(uint i = 0; i < b->ndof; i++)
			AddCLink(b->pos[i]);
	}
}

//-------------------------------------------------------------------------------------------------
// CalcCoef

void FixPosCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->endBody; b != handle->refBody; b = b->parent){
		for(uint i = 0; i < b->ndof; i++){
			((CLink*)links[idx + i])->SetCoef(b->Jv_abs[i] - b->Jw_abs[i] % (handle->endPoseAbs.Pos() - b->pivot));
			idx += b->ndof;
		}
	}
}

void FixOriCon::CalcCoef(){
	uint idx = 0;
	for(IKBody* b = handle->endBody; b != handle->refBody; b = b->parent){
		for(uint i = 0; i < b->ndof; i++){
			((CLink*)links[idx + i])->SetCoef(-1.0 * b->Jw_abs[i]);
			idx += b->ndof;
		}
	}
}

//-------------------------------------------------------------------------------------------------
// CalcError

void FixPosCon::CalcDeviation(){
	y = (handle->refPoseAbs.Ori() * handle->desPose.Pos() + handle->refPoseAbs.Pos()) - handle->endPoseAbs.Pos();
}

void FixOriCon::CalcDeviation(){
	quat_t qerror = handle->endPoseAbs.Ori().Conjugated() * (handle->refPoseAbs.Ori() * handle->desPose.Ori());
	y = handle->endPoseAbs.Ori() * (qerror.Theta() * qerror.Axis());
}

}
