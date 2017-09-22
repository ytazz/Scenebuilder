#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbikhandle.h>

#include <Foundation/UTPreciseTimer.h>
static UTPreciseTimer timer;

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

IKSolver::IKSolver(){
	numIter = 3;
	gravity = vec3_t(0.0, 0.0, -9.8);
	
	param.methodMajor = Solver::Method::Major::GaussNewton1;
	param.methodMinor = Solver::Method::Minor::Direct;

	bodyColor  .name = "black"  ; bodyColor  .Init();
	jointColor .name = "black"  ; jointColor .Init();
	handleColor.name = "magenta"; handleColor.Init();
	velColor   .name = "cyan"   ; velColor   .Init();
	angvelColor.name = "green"  ; angvelColor.Init();
	accColor   .name = "cyan"   ; accColor   .Init();
	angaccColor.name = "green"  ; angaccColor.Init();
	forceColor .name = "cyan"   ; forceColor .Init();
	momentColor.name = "green"  ; momentColor.Init();

	velScale    = 1.0f;
	angvelScale = 1.0f;
	accScale    = 1.0f;
	angaccScale = 1.0f;
	forceScale  = 0.01f;
	momentScale = 0.01f;

	showBody   = true;
	showJoint  = true;
	showHandle = true;
	showVel    = false;
	showAngvel = false;
	showAcc    = false;
	showAngacc = false;
	showForce  = false;
	showMoment = false;
	showTorque = false;

	ready = false;
}

IKBody* IKSolver::AddBody(){
	// 重複追加のチェックやループ形成チェックは行わないのでユーザが注意
	IKBody* body = new IKBody(this);
	ikBodies.push_back(body);
	ready = false;
	return body;
}

void IKSolver::DeleteBody(IKBody* body){
	for(uint i = 0; i < ikHandles.size(); i++){
		IKHandle* h = ikHandles[i];
		if(h->body == body)
			h->body = 0;
	}
	RemoveFromArray(ikBodies, body);
	ready = false;
}

IKJoint* IKSolver::AddJoint(int _type){
	IKJoint* joint = new IKJoint(this, _type);
	ikJoints.push_back(joint);
	ready = false;
	return joint;
}

void IKSolver::DeleteJoint(IKJoint* joint){
	RemoveFromArray(ikJoints, joint);
	ready = false;
}

IKHandle* IKSolver::AddHandle(IKBody* ikBody){
	IKHandle* handle = new IKHandle(this, ikBody);
	ikHandles.push_back(handle);
	ikBody->handles.push_back(handle);
	ready = false;
	return handle;
}

void IKSolver::DeleteHandle(IKHandle* handle){
	if(handle->body)
		RemoveFromArray(handle->body->handles, handle);
	RemoveFromArray(ikHandles, handle);
	ready = false;
}

IKJointHandle* IKSolver::AddJointHandle(IKJoint* ikJoint){
	IKJointHandle* handle = new IKJointHandle(this, ikJoint);
	ikJointHandles.push_back(handle);
	ready = false;
	return handle;
}

void IKSolver::DeleteJointHandle(IKJointHandle* handle){
	RemoveFromArray(ikJointHandles, handle);
	ready = false;
}

IKComHandle* IKSolver::AddComHandle(){
	IKComHandle* handle = new IKComHandle(this);
	ikComHandles.push_back(handle);
	ready = false;
	return handle;
}

void IKSolver::DeleteComHandle(IKComHandle* handle){
	RemoveFromArray(ikComHandles, handle);
	ready = false;
}

void IKSolver::Init(){
	timer.CountUS();

	for(uint i = 0; i < ikBodies      .size(); i++) ikBodies      [i]->Init();
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->Init();
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->Init();
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->Init();
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->Init();

	for(uint i = 0; i < ikBodies      .size(); i++) ikBodies      [i]->AddVar();
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->AddVar();
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->AddVar();
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->AddVar();
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->AddVar();

	for(uint i = 0; i < ikBodies      .size(); i++) ikBodies      [i]->AddCon();
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->AddCon();
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->AddCon();
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->AddCon();
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->AddCon();

	Solver::Init();
	ready = true;

	int timeInit = timer.CountUS();
}

void IKSolver::Prepare(){
	timer.CountUS();

	for(uint i = 0; i < ikBodies      .size(); i++) ikBodies      [i]->Prepare();
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->Prepare();
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->Prepare();
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->Prepare();
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->Prepare();

	timePrepare = timer.CountUS();
}

void IKSolver::Finish(){
	timer.CountUS();

	for(uint i = 0; i < ikBodies      .size(); i++) ikBodies      [i]->Finish();
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->Finish();
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->Finish();
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->Finish();
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->Finish();

	timeFinish = timer.CountUS();
}

void IKSolver::Update(){
	timer.CountUS();

	for(uint i = 0; i < ikBodies.size(); i++){
		if(!ikBodies[i]->parBody)
			ikBodies[i]->Update();
	}
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->Update();
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->Update();
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->Update();
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->Update();

	timeUpdate += timer.CountUS();
}

void IKSolver::CompPosIK(){
	if(!ready)
		Init();

	mode                 = Mode::Pos;
	param.minStepSize    = 0.0;
	param.maxStepSize    = 1.0;
	param.cutoffStepSize = 1.0e-10;
	param.hastyStepSize  = true;
	
	Prepare();
	timeUpdate = 0;
	for(int i = 0; i < numIter; i++)
		Solver::Step();
	Finish();
}

void IKSolver::CompVelIK(){
	if(!ready)
		Init();

	mode = Mode::Vel;
	param.minStepSize = 1.0;
	param.maxStepSize = 1.0;

	Prepare();
	Solver::Step();
	Finish();
}

void IKSolver::CompAccIK(){
	if(!ready)
		Init();

	mode = Mode::Acc;
	param.minStepSize = 1.0;
	param.maxStepSize = 1.0;

	Prepare();
	Solver::Step();
	Finish();
}

void IKSolver::CompForceIK(){
	if(!ready)
		Init();

	mode = Mode::Force;
	param.minStepSize = 1.0;
	param.maxStepSize = 1.0;

	Prepare();
	Solver::Step();
	Finish();
}

void IKSolver::CompInertia(const vec3_t& origin, mat3_t& I){
	I.clear();
	mat3_t R;
	mat3_t Ib;
	vec3_t r;
	mat3_t rc;
	for(uint i = 0; i < ikBodies.size(); i++){
		IKBody* body = ikBodies[i];
		r  = (body->pos - origin);
		rc = mat3_t::Cross(r);
		body->ori.ToMatrix(R);

		I += (R * body->inertia * R.trans() - body->mass * rc * rc);

	}
}

real_t IKSolver::CalcObjective(){
	Update();
	return Solver::CalcObjective();
}

void IKSolver::CalcDirection(){
	Update();
	Solver::CalcDirection();
}
	
void IKSolver::Draw(GRRenderIf* render){
	for(uint i = 0; i < ikBodies      .size(); i++) ikBodies      [i]->Draw(render);
	for(uint i = 0; i < ikJoints      .size(); i++) ikJoints      [i]->Draw(render);
	for(uint i = 0; i < ikHandles     .size(); i++) ikHandles     [i]->Draw(render);
	for(uint i = 0; i < ikJointHandles.size(); i++) ikJointHandles[i]->Draw(render);
	for(uint i = 0; i < ikComHandles  .size(); i++) ikComHandles  [i]->Draw(render);
}

}
