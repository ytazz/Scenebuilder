#include <sbsolver.h>
#include <sblink.h>

namespace Scenebuilder{;

Constraint::Constraint(Solver* solver, uint n){
	nelem	= n;
	index   = 0;
	enabled = true;
	active  = true;
	
	corrRate = 0.1;
	corrMax  = FLT_MAX;

	solver->AddCon(this);
}

SLink* Constraint::AddSLink(Variable* var, real_t coef){
	SLink* link = new SLink(var, this, coef);
	link->Connect();
	solver->links.push_back(link);
	return link;
}

XLink* Constraint::AddXLink(Variable* var){
	XLink* link = new XLink(var, this);
	link->Connect();
	solver->links.push_back(link);
	return link;
}

CLink* Constraint::AddCLink(Variable* var, vec3_t coef ){
	CLink* link = new CLink(var, this, coef);
	link->Connect();
	solver->links.push_back(link);
	return link;
}

RLink* Constraint::AddRLink(Variable* var){
	RLink* link = new RLink(var, this);
	link->Connect();
	solver->links.push_back(link);
	return link;
}

void Constraint::CalcError(){
	if(enabled){
		CalcDeviation();
		//for(uint k = 0; k < nelem; k++)
		//	e[k] = 0.5 * y[k] * y[k];
	}
	else{
		y.clear();
		//e.clear();
	}
}

void Constraint::CalcDeviation(){
	y.clear();
	for(uint i = 0; i < links.size(); i++)
		links[i]->AddError();
}

void Constraint::RegisterDeviation(vvec_t& yvec){
	for(uint n = 0; n < nelem; n++)
		yvec[index+n] = y[n];
}

void Constraint::ResetState(){
	dy.clear();
	l .clear();
	dl.clear();
}

void Constraint::CalcCorrection(){
	// ガウスザイデルで用いるJ*J^Tの対角成分 = Jの各行の二乗和を計算
	const real_t eps = (real_t)1.0e-10;
	
	J.clear();
	for(uint i = 0; i < links.size(); i++){
		Link* lnk = links[i];
		if(!lnk->var->locked)
			lnk->AddRowSqr(J);
	}
	
	// 対角成分の逆数
	for(uint k = 0; k < nelem; k++){
		Jinv[k] = (J[k] > eps ? (real_t)1.0/J[k] : (real_t)0.0);
	}

	// 拘束偏差yと拘束誤差eの修正量を設定
	// ただし修正幅は上限を超えないようにする
	dyd = -corrRate * y;
	const real_t dyd_lim = corrMax;
	real_t dyd_max = 0.0;
	for(uint k = 0; k < nelem; k++)
		dyd_max = std::max(dyd_max, std::abs(dyd[k]));
	
	if(dyd_max > dyd_lim)
		dyd *= (dyd_lim / dyd_max);

	for(uint k = 0; k < nelem; k++){
		for(uint i = 0; i < links.size(); i++){
			links[i]->Backward(k, dyd[k], &Variable::UpdateConjugate3);
		}
	}
}

void Constraint::UpdateGradient(uint k){
	for(uint i = 0; i < links.size(); i++){
		links[i]->Backward(k, -y[k], &Variable::UpdateVar2);
	}
}

void Constraint::UpdateMultiplier(uint k){
	// update multiplier
	real_t lnew;
	dl[k] = (real_t)-1.0 * Jinv[k] * (dy[k] - dyd[k]);
	lnew  = l[k] + dl[k];
	Project(lnew, k);
	dl[k] = lnew - l[k];
	l[k]  = lnew;

	// update variable
	if(solver->param.methodMinor == Solver::Method::Minor::GaussSeidel){
		for(uint i = 0; i < links.size(); i++){
			links[i]->Backward(k, dl[k], &Variable::UpdateVar1);
		}
	}
	else{
		for(uint i = 0; i < links.size(); i++){
			links[i]->Backward(k, dl[k], &Variable::UpdateVar2);
		}
	}
}

void Constraint::UpdateError1(uint k, real_t ddy){
	dy[k] += ddy;
}

void Constraint::UpdateError2(uint k, real_t _dy){
	dy[k] = _dy;
}

void Constraint::UpdateError3(uint k, real_t ddy){
	dy[k] += ddy;
	for(uint i = 0; i < links.size(); i++)
		links[i]->Backward(k, ddy, &Variable::UpdateConjugate1);
}

void Constraint::UpdateConjugate(uint k){
	for(Links::iterator it = links.begin(); it != links.end(); it++){
		Link* lnk = *it;
		lnk->Backward(k, dy[k], &Variable::UpdateConjugate1);
	}
}

//-------------------------------------------------------------------------------------------------

FixConS::FixConS(Solver* solver, SVar* var):Constraint(solver, 1){
	desired = 0.0;
	AddSLink(var, 1.0);
}

void FixConS::CalcDeviation(){
	y[0] = ((SVar*)links[0]->var)->val - desired;
}

//-------------------------------------------------------------------------------------------------

MatchConS::MatchConS(Solver* solver, SVar* var0, SVar* var1):Constraint(solver, 1){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

FixConV3::FixConV3(Solver* solver, V3Var* var):Constraint(solver, 3){
	AddSLink(var, 1.0);
}

void FixConV3::CalcDeviation(){
	y = ((V3Var*)links[0]->var)->val - desired;
}

//-------------------------------------------------------------------------------------------------

MatchConV3::MatchConV3(Solver* solver, V3Var* var0, V3Var* var1):Constraint(solver, 3){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

FixConQ::FixConQ(Solver* solver, QVar* var):Constraint(solver, 3){
	AddSLink(var, 1.0);
}

void FixConQ::CalcDeviation(){
	quat_t q0 = desired;
	quat_t q1 = ((QVar*)links[0]->var)->val;
	quat_t qerror = q0.Conjugated() * q1;
	y = q0 * (qerror.Theta() * qerror.Axis());
}

//-------------------------------------------------------------------------------------------------

MatchConQ::MatchConQ(Solver* solver, QVar* var0, QVar* var1):Constraint(solver, 3){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

void MatchConQ::CalcDeviation(){
	quat_t q0 = ((QVar*)links[0]->var)->val;
	quat_t q1 = ((QVar*)links[1]->var)->val;
	quat_t qerror = q0.Conjugated() * q1;
	y = q0 * (qerror.Theta() * qerror.Axis());
}

//-------------------------------------------------------------------------------------------------

RangeConS::RangeConS(Solver* solver, SVar* var):Constraint(solver, 1){
	AddSLink(var, 1.0);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConS::CalcDeviation(){
	real_t s = ((SVar*)links[0]->var)->val;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConS::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

DiffConS::DiffConS(Solver* solver, SVar* var0, SVar* var1):Constraint(solver, 1){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void DiffConS::CalcDeviation(){
	real_t diff = ((SVar*)links[1]->var)->val - ((SVar*)links[0]->var)->val;
	on_lower = (diff < _min);
	on_upper = (diff > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = diff - _min;
	if(on_upper)
		y[0] = diff - _max;
}

void DiffConS::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

}
