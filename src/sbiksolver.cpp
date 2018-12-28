#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikjoint.h>
#include <sbikhandle.h>
#include <sbmessage.h>

#include <Foundation/UTQPTimer.h>
static UTQPTimer timer;

namespace Scenebuilder{;

static const real_t inf = numeric_limits<real_t>::max();

//-------------------------------------------------------------------------------------------------

IKSolver::IKSolver(){
	numIter = 3;
	gravity = vec3_t(0.0, 0.0, -9.8);
	
	param.methodMajor = Solver::Method::Major::GaussNewton;
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

IKBody* IKSolver::AddBody(const string& _name){
	// 重複追加のチェックやループ形成チェックは行わないのでユーザが注意
	UTRef<IKBody> body(new IKBody(this, _name));
	ikBodies.push_back(body);
	ready = false;
	return body;
}

void IKSolver::DeleteBody(IKBody* body){
	for(uint i = 0; i < ikHandles.size(); i++){
		auto& h = ikHandles[i];
		if(h->body == body)
			h->body = 0;
	}
	RemoveFromArray(ikBodies, body);
	ready = false;
}

IKJoint* IKSolver::AddJoint(int _type, const string& _name){
	UTRef<IKJoint> joint(new IKJoint(this, _type, _name));
	ikJoints.push_back(joint);
	ready = false;
	return joint;
}

void IKSolver::DeleteJoint(IKJoint* joint){
	RemoveFromArray(ikJoints, joint);
	ready = false;
}

IKMate* IKSolver::AddMate(int _type, const string& _name){
	UTRef<IKMate> mate(new IKMate(this, _type, _name));
	ikMates.push_back(mate);
	ready = false;
	return mate;
}

void IKSolver::DeleteMate(IKMate* mate){
	RemoveFromArray(ikMates, mate);
	ready = false;
}

IKLimit* IKSolver::AddLimit(int _type, const string& _name){
	UTRef<IKLimit> limit(new IKLimit(this, _type, _name));
	ikLimits.push_back(limit);
	ready = false;
	return limit;
}

void IKSolver::DeleteLimit(IKLimit* limit){
	RemoveFromArray(ikLimits, limit);
	ready = false;
}

IKHandle* IKSolver::AddHandle(IKBody* ikBody, const string& _name){
	UTRef<IKHandle> handle(new IKHandle(this, ikBody, _name));
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

IKJointHandle* IKSolver::AddJointHandle(IKJoint* ikJoint, const string& _name){
	UTRef<IKJointHandle> handle(new IKJointHandle(this, ikJoint, _name));
	ikJointHandles.push_back(handle);
	ready = false;
	return handle;
}

void IKSolver::DeleteJointHandle(IKJointHandle* handle){
	RemoveFromArray(ikJointHandles, handle);
	ready = false;
}

IKComHandle* IKSolver::AddComHandle(const string& _name){
	UTRef<IKComHandle> handle(new IKComHandle(this, _name));
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

	for(auto& body        : ikBodies      ) body       ->Init();
	for(auto& joint       : ikJoints      ) joint      ->Init();
	for(auto& mate        : ikMates       ) mate       ->Init();
	for(auto& limit       : ikLimits      ) limit      ->Init();
	for(auto& handle      : ikHandles     ) handle     ->Init();
	for(auto& jointHandle : ikJointHandles) jointHandle->Init();
	for(auto& comHandle   : ikComHandles  ) comHandle  ->Init();

	for(auto& body        : ikBodies      ) body       ->AddVar();
	for(auto& joint       : ikJoints      ) joint      ->AddVar();
	for(auto& mate        : ikMates       ) mate       ->AddVar();
	for(auto& limit       : ikLimits      ) limit      ->AddVar();
	for(auto& handle      : ikHandles     ) handle     ->AddVar();
	for(auto& jointHandle : ikJointHandles) jointHandle->AddVar();
	for(auto& comHandle   : ikComHandles  ) comHandle  ->AddVar();

	for(auto& body        : ikBodies      ) body       ->AddCon();
	for(auto& joint       : ikJoints      ) joint      ->AddCon();
	for(auto& mate        : ikMates       ) mate       ->AddCon();
	for(auto& limit       : ikLimits      ) limit      ->AddCon();
	for(auto& handle      : ikHandles     ) handle     ->AddCon();
	for(auto& jointHandle : ikJointHandles) jointHandle->AddCon();
	for(auto& comHandle   : ikComHandles  ) comHandle  ->AddCon();

	Solver::Init();
	Solver::SetCorrection(ID(), 1.0);

	ready = true;

	int timeInit = timer.CountUS();
}

void IKSolver::Reset(){
	if(!ready)
		Init();

	timer.CountUS();

	for(auto& body  : ikBodies) body ->Reset();
	for(auto& joint : ikJoints) joint->Reset();
	for(auto& mate  : ikMates ) mate ->Reset();
	
	timeReset = timer.CountUS();
}

void IKSolver::Prepare(){
	timer.CountUS();

	for(auto& body        : ikBodies      ) body       ->Prepare();
	for(auto& joint       : ikJoints      ) joint      ->Prepare();
	for(auto& mate        : ikMates       ) mate       ->Prepare();
	for(auto& limit       : ikLimits      ) limit      ->Prepare();
	for(auto& handle      : ikHandles     ) handle     ->Prepare();
	for(auto& jointHandle : ikJointHandles) jointHandle->Prepare();
	for(auto& comHandle   : ikComHandles  ) comHandle  ->Prepare();

	timePrepare = timer.CountUS();
}

void IKSolver::Finish(){
	timer.CountUS();

	for(auto& body        : ikBodies      ) body       ->Finish();
	for(auto& joint       : ikJoints      ) joint      ->Finish();
	for(auto& mate        : ikMates       ) mate       ->Finish();
	for(auto& limit       : ikLimits      ) limit      ->Finish();
	for(auto& handle      : ikHandles     ) handle     ->Finish();
	for(auto& jointHandle : ikJointHandles) jointHandle->Finish();
	for(auto& comHandle   : ikComHandles  ) comHandle  ->Finish();

	timeFinish = timer.CountUS();
}

void IKSolver::Update(){
	timer.CountUS();

	for(auto& joint : ikJoints) joint->Limit();
	
	for(auto& body : ikBodies){
		if(!body->parBody)
			body->Update();
	}
	
	for(auto& joint       : ikJoints      ) joint      ->Update();
	for(auto& mate        : ikMates       ) mate       ->Update();
	for(auto& limit       : ikLimits      ) limit      ->Update();
	for(auto& handle      : ikHandles     ) handle     ->Update();
	for(auto& jointHandle : ikJointHandles) jointHandle->Update();
	for(auto& comHandle   : ikComHandles  ) comHandle  ->Update();

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
	for(int i = 0; i < numIter; i++){
		Solver::Step();
	}
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
	for(auto& body : ikBodies){
		r  = (body->pos - origin);
		rc = mat3_t::Cross(r);
		body->ori.ToMatrix(R);

		I += (R * body->inertia * R.trans() - body->mass * rc * rc);
	}
}

real_t IKSolver::CalcObjective(){
	Update();

	real_t obj = Solver::CalcObjective();

	for(IKLimit* lim : ikLimits){
		obj += 1000000000.0*lim->CalcError();
	}

	return obj;
}

void IKSolver::CalcDirection(){
	Update();
	Solver::CalcDirection();
}
	
void IKSolver::Draw(GRRenderIf* render){
	for(auto& body        : ikBodies      ) body       ->Draw(render);
	for(auto& joint       : ikJoints      ) joint      ->Draw(render);
	for(auto& mate        : ikMates       ) mate       ->Draw(render);
	for(auto& handle      : ikHandles     ) handle     ->Draw(render);
	for(auto& jointHandle : ikJointHandles) jointHandle->Draw(render);
	for(auto& comHandle   : ikComHandles  ) comHandle  ->Draw(render);
}

}
