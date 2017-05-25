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

	bodyColor  .name = "cyan"   ; bodyColor  .Init();
	velColor   .name = "cyan"   ; velColor   .Init();
	accColor   .name = "cyan"   ; accColor   .Init();
	forceColor .name = "cyan"   ; forceColor .Init();
	handleColor.name = "magenta"; handleColor.Init();

	velScale   = 1.0f;
	accScale   = 1.0f;
	forceScale = 0.01f;

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

	if(mode == Mode::Force){
		forceTotal .clear();
		momentTotal.clear();
		for(uint i = 0; i < ikBodies.size(); i++){
			IKBody* body = ikBodies[i];
			forceTotal  += body->force;
			momentTotal += body->moment;
		}
	}

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
	param.numIterMajor   = numIter;
	param.methodStepSize = Method::StepSize::MinOrMax;
	param.minStepSize    = 0.0;
	param.maxStepSize    = 1.0;
	
	Prepare();
	timeUpdate = 0;
	Solver::Solve();
	Finish();
}

void IKSolver::CompVelIK(){
	if(!ready)
		Init();

	mode = Mode::Vel;
	param.numIterMajor   = 1;
	param.methodStepSize = Method::StepSize::Max;
	param.maxStepSize    = 1.0;

	Prepare();
	Solver::Solve();
	Finish();
}

void IKSolver::CompAccIK(){
	if(!ready)
		Init();

	mode = Mode::Acc;
	param.numIterMajor   = 1;
	param.methodStepSize = Method::StepSize::Max;
	param.maxStepSize    = 1.0;

	Prepare();
	Solver::Solve();
	Finish();
}

void IKSolver::CompForceIK(){
	if(!ready)
		Init();

	mode = Mode::Force;
	param.numIterMajor   = 1;
	param.methodStepSize = Method::StepSize::Max;
	param.maxStepSize    = 1.0;

	Prepare();
	Solver::Solve();
	Finish();
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
