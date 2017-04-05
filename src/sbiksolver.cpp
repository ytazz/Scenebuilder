#include <sbiksolver.h>
#include <sbikbody.h>
#include <sbikhandle.h>

namespace Scenebuilder{;

//-------------------------------------------------------------------------------------------------

IKSolver::IKSolver(){
	param.numIterMajor = 10;
	param.numIterMinor = 10;
	param.minStepSize  = 1.0e-10;
	param.maxStepSize  = 1.0;
					   
	dmaxRevolutive     = 0.1;        
	dmaxPrismatic      = 0.1;
	
	ready = false;
}

void IKSolver::Init(){
	Solver::Init();
}

IKBody* IKSolver::AddBody(IKBody* par){
	// 重複追加のチェックやループ形成チェックは行わないのでユーザが注意
	IKBody* body = new IKBody(this);
	ikBodies.push_back(body);
	if(par){
		par->children.push_back(body);
		body->parent = par;
	}
	return body;
}

void IKSolver::DeleteBody(IKBody* body){
	for(uint i = 0; i < ikHandles.size(); ){
		IKHandle* h = ikHandles[i];
		if(h->endBody == body)
			h->endBody = 0;
		if(h->refBody == body)
			h->refBody = 0;
		else i++;
	}
	body->DeleteVars();
	body->DeleteCons();
	RemoveFromArray(ikBodies, body);
}

IKHandle* IKSolver::AddHandle(IKBody* endBody, IKBody* refBody){
	IKHandle* handle = new IKHandle(this, endBody, refBody);
	ikHandles.push_back(handle);
	return handle;
}

void IKSolver::DeleteHandle(IKHandle* handle){
	handle->DeleteCons();
	RemoveFromArray(ikHandles, handle);
}

void IKSolver::Prepare(){
	for(uint i = 0; i < ikBodies.size(); i++){
		ikBodies[i]->handled = false;
	}
	for(uint i = 0; i < ikHandles.size(); i++){
		ikHandles[i]->endBody->MarkHandled(ikHandles[i]->refBody);
	}

	for(uint i = 0; i < ikBodies.size(); i++){
		if(!ikBodies[i]->parent)
			ikBodies[i]->Prepare();
	}
	for(uint i = 0; i < ikHandles.size(); i++){
		ikHandles[i]->Prepare();
	}
}

void IKSolver::CompIK(){
	if(ikHandles.empty())
		return;

	Prepare();
	Solver::Solve();
	Prepare();
}

void IKSolver::CompFK(){
	for(uint i = 0; i < ikBodies.size(); i++){
		if(!ikBodies[i]->parent)
			ikBodies[i]->CompFK();
	}
	for(uint i = 0; i < ikHandles.size(); i++){
		ikHandles[i]->CompFK();
	}
}

real_t IKSolver::CalcObjective(){
	Prepare();
	
	return Solver::CalcObjective();
}

void IKSolver::CalcInitialValue(){

}

void IKSolver::CalcCoef(){

}
	
void IKSolver::SetBodyColor(const Vec4f& c){
	bodyColor = c;
}

void IKSolver::SetHandleColor(const Vec4f& c){
	handleColor = c;
}

void IKSolver::Draw(GRRenderIf* render){
	for(uint i = 0; i < ikBodies.size(); i++)
		ikBodies[i]->Draw(render);
	for(uint i = 0; i < ikHandles.size(); i++)
		ikHandles[i]->Draw(render);
}

}
