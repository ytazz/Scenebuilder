#include <sbsolver.h>
#include <sblink.h>

namespace Scenebuilder{;

static const real_t pi  = M_PI;
static const real_t eps = 1.0e-10;

Constraint::Constraint(Solver* solver, uint n, ID _id, int _type, real_t _scale):ID(_id){
    type       = _type;
	nelem	   = n;
	level      = 0;
	index      = 0;
	enabled    = true;
	active     = true;
	weight     = vec3_t(1.0, 1.0, 1.0);
    //lambda     = vec3_t(1.0, 1.0, 1.0);
	scale      = _scale;
	scale2     = scale * scale;
	scale_inv  = 1.0 / scale;
	scale2_inv = scale_inv * scale_inv;
	corrRate   = 0.1;
	corrMax    = FLT_MAX;

	solver->AddCon(this);
}

SLink* Constraint::AddSLink(Variable* var, real_t coef){
	UTRef<SLink> link(new SLink(var, this, coef));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

C2Link* Constraint::AddC2Link(Variable* var){
	UTRef<C2Link> link(new C2Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

R2Link* Constraint::AddR2Link(Variable* var){
	UTRef<R2Link> link(new R2Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

M2Link* Constraint::AddM2Link(Variable* var){
	UTRef<M2Link> link(new M2Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

X3Link* Constraint::AddX3Link(Variable* var){
	UTRef<X3Link> link(new X3Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

C3Link* Constraint::AddC3Link(Variable* var){
	UTRef<C3Link> link(new C3Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

R3Link* Constraint::AddR3Link(Variable* var){
	UTRef<R3Link> link(new R3Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

M3Link* Constraint::AddM3Link(Variable* var){
	UTRef<M3Link> link(new M3Link(var, this));
	link->Connect();
	solver->links.push_back(link);
	return link;
}

void Constraint::SetPriority(uint newlv){
	level = newlv;
}

void Constraint::CalcError(){
	if(enabled){
		CalcDeviation();

        if( type == Type::Equality ||
            type == Type::InequalityPenalty ){
            // quadratic cost
		    for(int k = 0; k < nelem; k++)
			    e[k] = 0.5 * weight[k]*weight[k]*y[k]*y[k];
        }
        if( type == Type::InequalityBarrier ){
            // logarithmic cost
            for(int k = 0; k < nelem; k++){
			    e[k] = -solver->param.complRelaxation*log(std::max(eps, y[k]));
                e[k] = std::max(0.0, e[k]);
            }
        }
	}
	else{
		y.clear();
		e.clear();
	}
}

void Constraint::CalcDeviation(){
	y.clear();
	for(Link* link : links)
		link->AddError();
}

void Constraint::RegisterCorrection(vvec_t& dydvec){
	for(int i = 0; i < nelem; i++)
		dydvec[index+i] = weight[i] * dyd[i];
}

void Constraint::RegisterDeviation(vvec_t& yvec){
	for(int i = 0; i < nelem; i++)
		yvec[index+i] = weight[i] * y[i];
}

void Constraint::ResetState(){
	dy.clear();
	l .clear();
	dl.clear();
}

void Constraint::CalcCorrection(){
	// ガウスザイデルで用いるJ*J^Tの対角成分 = Jの各行の二乗和を計算
	
    J.clear();
	for(Link* link : links_active){
		link->AddRowSqr(J);
	}
	
	// 対角成分の逆数
	for(int k = 0; k < nelem; k++){
		Jinv[k] = (J[k] > eps ? (real_t)1.0/J[k] : (real_t)0.0);
	}

	// 拘束偏差yの修正量を設定
	if( type == Type::Equality ||
        type == Type::InequalityPenalty ){

        // negative gradient of quadratic cost times correction rate
	    dyd = -corrRate * y;
    }
    if( type == Type::InequalityBarrier ){
        dyd =  corrRate * y;
        //for(int k = 0; k < nelem; k++){
        //    // negative gradient of logarithmic cost times correction rate
        //    dyd[k] = corrRate * solver->param.complRelaxation/y[k];
        //}
    }

	// ただし修正幅は上限を超えないようにする
    const real_t dyd_lim = corrMax;
	real_t dyd_max = 0.0;
	for(int k = 0; k < nelem; k++)
		dyd_max = std::max(dyd_max, std::abs(dyd[k]));
	
	if(dyd_max > dyd_lim)
		dyd *= (dyd_lim / dyd_max);
    
	for(int k = 0; k < nelem; k++){
		for(Link* link : links_active){
			link->ColTrans(k, dyd[k], &Variable::UpdateConjugate3);
		}
	}
}

void Constraint::UpdateGradient(uint k){
	for(Link* link : links_active){
		link->ColTrans(k, -y[k], &Variable::UpdateVar2);
	}
}

void Constraint::UpdateMultiplier(uint k){
	dy[k] = 0.0;
	for(Link* link : links_active){
		link->Row(k, link->var->dx, &Constraint::UpdateError1);
	}

	// update multiplier
	real_t lnew;
	dl[k] = (real_t)-1.0 * Jinv[k] * (dy[k] - dyd[k]);
	lnew  = l[k] + dl[k];
	Project(lnew, k);
	dl[k] = lnew - l[k];
	l[k]  = lnew;

	// update variable
	if(solver->param.methodMinor == Solver::Method::Minor::GaussSeidel){
		for(Link* link : links_active){
			link->ColTrans(k, dl[k], &Variable::UpdateVar1);
		}
	}
	else{
		for(Link* link : links_active){
			link->ColTrans(k, dl[k], &Variable::UpdateVar2);
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
	for(Link* link : links_active)
		link->ColTrans(k, ddy, &Variable::UpdateConjugate1);
}

void Constraint::UpdateConjugate(uint k){
	for(Link* link : links_active){
		link->ColTrans(k, dy[k], &Variable::UpdateConjugate1);
	}
}

//-------------------------------------------------------------------------------------------------

FixConS::FixConS(Solver* solver, ID id, SVar* var, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
	desired = 0.0;
	AddSLink(var, 1.0);
}

void FixConS::CalcDeviation(){
	y[0] = ((SVar*)links[0]->var)->val - desired;
}

//-------------------------------------------------------------------------------------------------

FixConV2::FixConV2(Solver* solver, ID id, V2Var* var, real_t _scale):Constraint(solver, 2, id, Constraint::Type::Equality, _scale){
	AddSLink(var, 1.0);
}

void FixConV2::CalcDeviation(){
	y[0] = ((V2Var*)links[0]->var)->val[0] - desired[0];
	y[1] = ((V2Var*)links[0]->var)->val[1] - desired[1];
}

//-------------------------------------------------------------------------------------------------

FixConV3::FixConV3(Solver* solver, ID id, V3Var* var, real_t _scale):Constraint(solver, 3, id, Constraint::Type::Equality, _scale){
	AddSLink(var, 1.0);
}

void FixConV3::CalcDeviation(){
	y = ((V3Var*)links[0]->var)->val - desired;
}

//-------------------------------------------------------------------------------------------------

FixConQ::FixConQ(Solver* solver, ID id, QVar* var, real_t _scale):Constraint(solver, 3, id, Constraint::Type::Equality, _scale){
	AddSLink(var, 1.0);
}

void FixConQ::CalcDeviation(){
	quat_t q0 = desired;
	quat_t q1 = ((QVar*)links[0]->var)->val;
	quat_t qerror = q0.Conjugated() * q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = q0 * (theta * axis);
}

//-------------------------------------------------------------------------------------------------

MatchConS::MatchConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

MatchConV2::MatchConV2(Solver* solver, ID id, V2Var* var0, V2Var* var1, real_t _scale):Constraint(solver, 2, id, Constraint::Type::Equality, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

MatchConV3::MatchConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale):Constraint(solver, 3, id, Constraint::Type::Equality, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

//-------------------------------------------------------------------------------------------------

MatchConQ::MatchConQ(Solver* solver, ID id, QVar* var0, QVar* var1, real_t _scale):Constraint(solver, 3, id, Constraint::Type::Equality, _scale){
	AddSLink(var0, -1.0);
	AddSLink(var1,  1.0);
}

void MatchConQ::CalcDeviation(){
	quat_t q0 = ((QVar*)links[0]->var)->val;
	quat_t q1 = ((QVar*)links[1]->var)->val;
	quat_t qerror = q0.Conjugated() * q1;
	vec3_t axis   = qerror.Axis ();
	real_t theta  = qerror.Theta();
	if(theta > pi)
		theta -= 2*pi;
	y = q0 * (theta * axis);
}

//-------------------------------------------------------------------------------------------------

RangeConS::RangeConS(Solver* solver, ID id, SVar* var, real_t _scale):Constraint(solver, 1, id, Constraint::Type::InequalityPenalty, _scale){
	AddSLink(var, 1.0);
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConS::CalcDeviation(){
	real_t s = ((SVar*)links[0]->var)->val;
	on_lower = (s <= _min);
	on_upper = (s >= _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConS::Project(real_t& l, uint k){
	if(_min == _max)
		return;
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

RangeConV2::RangeConV2(Solver* solver, ID id, V2Var* var, const vec2_t& _dir, real_t _scale):Constraint(solver, 1, id, Constraint::Type::InequalityPenalty, _scale){
	dir = _dir;

	AddR2Link(var);
	((R2Link*)links[0])->SetCoef(dir);

	real_t inf = numeric_limits<real_t>::max();
	_min     = -inf;
	_max     =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConV2::CalcDeviation(){
	active = false;
	real_t s = dir*((V2Var*)links[0]->var)->val;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active |= on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConV2::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

RangeConV3::RangeConV3(Solver* solver, ID id, V3Var* var, const vec3_t& _dir, real_t _scale):Constraint(solver, 1, id, Constraint::Type::InequalityPenalty, _scale){
	dir = _dir;

	AddR3Link(var);
	((R3Link*)links[0])->SetCoef(dir);

	real_t inf = numeric_limits<real_t>::max();
	_min     = -inf;
	_max     =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConV3::CalcDeviation(){
	active = false;
	real_t s = dir*((V3Var*)links[0]->var)->val;
	on_lower = (s < _min);
	on_upper = (s > _max);
	active |= on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConV3::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

DiffConS::DiffConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
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
	if(_min == _max)
		return;
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

FixConPlane::FixConPlane(Solver* solver, ID id, V3Var* var, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
	AddR3Link(var);
}

void FixConPlane::CalcCoef(){
	((R3Link*)links[0])->SetCoef(normal);
}

void FixConPlane::CalcDeviation(){
	y[0] = normal * ( ((V3Var*)links[0]->var)->val - origin);
}

//-------------------------------------------------------------------------------------------------

RangeConPlane::RangeConPlane(Solver* solver, ID id, V3Var* var, real_t _scale):Constraint(solver, 1, id, Constraint::Type::InequalityPenalty, _scale){
	AddR3Link(var);
	normal = vec3_t(0.0, 1.0, 0.0);
	origin = vec3_t();
	real_t inf = numeric_limits<real_t>::max();
	_min = -inf;
	_max =  inf;
	on_lower = false;
	on_upper = false;
}

void RangeConPlane::CalcCoef(){
	((R3Link*)links[0])->SetCoef(normal);
}

void RangeConPlane::CalcDeviation(){
	real_t s = normal * ( ((V3Var*)links[0]->var)->val - origin);
	on_lower = (s <= _min);
	on_upper = (s >= _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = s - _min;
	if(on_upper)
		y[0] = s - _max;
}

void RangeConPlane::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

ComplConS::ComplConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
	AddSLink(var0);
	AddSLink(var1);
}

void ComplConS::CalcCoef(){
	((SLink*)links[0])->SetCoef(((SVar*)links[1]->var)->val);
	((SLink*)links[1])->SetCoef(((SVar*)links[0]->var)->val);
}

void ComplConS::CalcDeviation(){
	y[0] = ((SVar*)links[0]->var)->val * ((SVar*)links[1]->var)->val;
}

//-------------------------------------------------------------------------------------------------

DistanceConV3::DistanceConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale):Constraint(solver, 1, id, Constraint::Type::InequalityPenalty, _scale){
	AddR3Link(var0);
	AddR3Link(var1);
}

void DistanceConV3::CalcCoef(){
	diff = ((V3Var*)links[0]->var)->val - ((V3Var*)links[1]->var)->val;
	diff_norm = diff.norm();
	if(diff_norm < eps){
		((R3Link*)links[0])->SetCoef( diff);
		((R3Link*)links[1])->SetCoef(-diff);
	}
	else{
		((R3Link*)links[0])->SetCoef( diff/diff_norm);
		((R3Link*)links[1])->SetCoef(-diff/diff_norm);
	}
}

void DistanceConV3::CalcDeviation(){
	on_lower = (diff_norm < _min);
	on_upper = (diff_norm > _max);
	active = on_lower | on_upper;
	if(on_lower)
		y[0] = diff_norm - _min;
	if(on_upper)
		y[0] = diff_norm - _max;
}

void DistanceConV3::Project(real_t& l, uint k){
	if(on_upper && l > 0.0)
		l = 0.0;
	if(on_lower && l < 0.0)
		l = 0.0;
	if(!on_upper && !on_lower)
		l = 0.0;
}

//-------------------------------------------------------------------------------------------------

C0ConS::C0ConS(Solver* solver, ID id, SVar* p0, SVar* v0, SVar* p1, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
	AddSLink(p1);
	AddSLink(p0);
	AddSLink(v0);
}
void C0ConS::CalcCoef(){
	uint i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	((SLink*)links[i++])->SetCoef(-h  );
}

//-------------------------------------------------------------------------------------------------

C0ConV3::C0ConV3(Solver* solver, ID id, V3Var* p0, V3Var* v0, V3Var* p1, real_t _scale):Constraint(solver, 3, id, Constraint::Type::Equality, _scale){
	AddSLink(p1);
	AddSLink(p0);
	AddSLink(v0);
}
void C0ConV3::CalcCoef(){
	uint i = 0;
	((SLink*)links[i++])->SetCoef( 1.0);
	((SLink*)links[i++])->SetCoef(-1.0);
	((SLink*)links[i++])->SetCoef(-h  );
}

//-------------------------------------------------------------------------------------------------

C1ConS::C1ConS(Solver* solver, ID id, SVar* p0, SVar* v0, SVar* a0, SVar* p1, real_t _scale):Constraint(solver, 1, id, Constraint::Type::Equality, _scale){
	AddSLink(p1);
	AddSLink(p0);
	AddSLink(v0);
	AddSLink(a0);
}
void C1ConS::CalcCoef(){
	uint i = 0;
	((SLink*)links[i++])->SetCoef( 1.0    );
	((SLink*)links[i++])->SetCoef(-1.0    );
	((SLink*)links[i++])->SetCoef(-h      );
	((SLink*)links[i++])->SetCoef(-0.5*h*h);
}

//-------------------------------------------------------------------------------------------------

C1ConV3::C1ConV3(Solver* solver, ID id, V3Var* p0, V3Var* v0, V3Var* a0, V3Var* p1, real_t _scale):Constraint(solver, 3, id, Constraint::Type::Equality, _scale){
	AddSLink(p1);
	AddSLink(p0);
	AddSLink(v0);
	AddSLink(a0);
}
void C1ConV3::CalcCoef(){
	uint i = 0;
	((SLink*)links[i++])->SetCoef( 1.0    );
	((SLink*)links[i++])->SetCoef(-1.0    );
	((SLink*)links[i++])->SetCoef(-h      );
	((SLink*)links[i++])->SetCoef(-0.5*h*h);
}

}
