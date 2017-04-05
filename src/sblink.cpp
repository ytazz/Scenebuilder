#include <sblink.h>

namespace Scenebuilder{;

void Link::Connect(){
	var->links.push_back(this);
	con->links.push_back(this);
}

//-------------------------------------------------------------------------------------------------

SLink::SLink(Variable* v, Constraint* c, real_t k):Link(v, c){
	SetCoef(k);
}

void SLink::SetCoef(real_t k){
	coef    = k;
	coefsqr = k*k;
}

void SLink::AddRowSqr(vec3_t& v){
	for(uint k = 0; k < con->nelem; k++)
		v[k] += coefsqr;
}

void SLink::AddColSqr(vec3_t& v){
	for(uint k = 0; k < var->nelem; k++)
		v[k] += coefsqr;
}

void SLink::AddError(){
	if(con->nelem == 1)
		 con->y[0] += coef * ((SVar*)var)->val;
	else con->y += coef * ((V3Var*)var)->val;
}
	
/*void SLink::UpdateVar(uint k, real_t dl){
	var->UpdateVar(k, coef * dl);
}
void SLink::UpdateGradient(uint k, real_t de){
	var->UpdateGradient(k, coef * de);
}
void SLink::UpdateError(uint k, real_t ddx, bool propagate){
	con->UpdateError(k, coef * ddx, propagate);
}*/
void SLink::Forward(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(k, coef * d);
}
void SLink::Backward(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(k, coef * d);
}

//-------------------------------------------------------------------------------------------------

void V3Link::SetCoef(vec3_t k){
	coef = k;
	for(uint i = 0; i < 3; i++)
		coefsqr[i] = k[i]*k[i];
}

//-------------------------------------------------------------------------------------------------

void XLink::AddRowSqr(vec3_t& v){
	v[0] += (coefsqr[2] + coefsqr[1]);
	v[1] += (coefsqr[2] + coefsqr[0]);
	v[2] += (coefsqr[1] + coefsqr[0]);
}

void XLink::AddColSqr(vec3_t& v){
	v[0] += coefsqr[2] + coefsqr[1];
	v[1] += coefsqr[2] + coefsqr[0];
	v[2] += coefsqr[1] + coefsqr[0];
}

void XLink::AddError(){
	con->y += coef % ((V3Var*)var)->val;
}

/*void XLink::UpdateVar(uint k, real_t dl){
	if(k == 0){
		var->UpdateVar(1, -coef[2] * dl);
		var->UpdateVar(2,  coef[1] * dl);
	}
	else if(k == 1){
		var->UpdateVar(0,  coef[2] * dl);
		var->UpdateVar(2, -coef[0] * dl);
	}
	else{
		var->UpdateVar(0, -coef[1] * dl);
		var->UpdateVar(1,  coef[0] * dl);
	}
}
void XLink::UpdateGradient(uint k, real_t dde){
	if(k == 0){
		var->UpdateGradient(1, -coef[2] * dde);
		var->UpdateGradient(2,  coef[1] * dde);
	}
	else if(k == 1){
		var->UpdateGradient(0,  coef[2] * dde);
		var->UpdateGradient(2, -coef[0] * dde);
	}
	else{
		var->UpdateGradient(0, -coef[1] * dde);
		var->UpdateGradient(1,  coef[0] * dde);
	}
}
void XLink::UpdateError(uint k, real_t ddx, bool propagate){
	if(k == 0){
		con->UpdateError(1,  coef[2] * ddx, propagate);
		con->UpdateError(2, -coef[1] * ddx, propagate);
	}
	else if(k == 1){
		con->UpdateError(0, -coef[2] * ddx, propagate);
		con->UpdateError(2,  coef[0] * ddx, propagate);
	}
	else{
		con->UpdateError(0,  coef[1] * ddx, propagate);
		con->UpdateError(1, -coef[0] * ddx, propagate);
	}
}*/
void XLink::Forward(uint k, real_t d, Constraint::UpdateFunc func){
	if(k == 0){
		(con->*func)(1,  coef[2] * d);
		(con->*func)(2, -coef[1] * d);
	}
	else if(k == 1){
		(con->*func)(0, -coef[2] * d);
		(con->*func)(2,  coef[0] * d);
	}
	else{
		(con->*func)(0,  coef[1] * d);
		(con->*func)(1, -coef[0] * d);
	}
}
void XLink::Backward(uint k, real_t d, Variable::UpdateFunc func){
	if(k == 0){
		(var->*func)(1, -coef[2] * d);
		(var->*func)(2,  coef[1] * d);
	}
	else if(k == 1){
		(var->*func)(0,  coef[2] * d);
		(var->*func)(2, -coef[0] * d);
	}
	else{
		(var->*func)(0, -coef[1] * d);
		(var->*func)(1,  coef[0] * d);
	}
}

//-------------------------------------------------------------------------------------------------

void CLink::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0];
	v[1] += coefsqr[1];
	v[2] += coefsqr[2];
}

void CLink::AddColSqr(vec3_t& v){
	v[0] += coefsqr[0];
	v[0] += coefsqr[1];
	v[0] += coefsqr[2];
}

void CLink::AddError(){
	con->y += coef * ((SVar*)var)->val;
}

/*void CLink::UpdateVar(uint k, real_t dl){
	var->UpdateVar(0, coef[k] * dl);
}
void CLink::UpdateGradient(uint k, real_t dde){
	var->UpdateGradient(0, coef[k] * dde);
}
void CLink::UpdateError(uint k, real_t ddx, bool propagate){
	con->UpdateError(0, coef[0] * ddx, propagate);
	con->UpdateError(1, coef[1] * ddx, propagate);
	con->UpdateError(2, coef[2] * ddx, propagate);
}*/
void CLink::Forward(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0] * d);
	(con->*func)(1, coef[1] * d);
	(con->*func)(2, coef[2] * d);
}
void CLink::Backward(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(0, coef[k] * d);
}

//-------------------------------------------------------------------------------------------------

void RLink::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0];
	v[0] += coefsqr[1];
	v[0] += coefsqr[2];
}

void RLink::AddColSqr(vec3_t& v){
	v[0] += coefsqr[0];
	v[1] += coefsqr[1];
	v[2] += coefsqr[2];
}

void RLink::AddError(){
	con->y[0] += coef * ((V3Var*)var)->val;
}

/*void RLink::UpdateVar(uint k, real_t dl){
	var->UpdateVar(0, coef[0] * dl);
	var->UpdateVar(1, coef[1] * dl);
	var->UpdateVar(2, coef[2] * dl);
}
void RLink::UpdateGradient(uint k, real_t dde){
	var->UpdateGradient(0, coef[0] * dde);
	var->UpdateGradient(1, coef[1] * dde);
	var->UpdateGradient(2, coef[2] * dde);
}
void RLink::UpdateError(uint k, real_t ddx, bool propagate){
	con->UpdateError(0, coef[k] * ddx, propagate);
}*/
void RLink::Forward(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[k] * d);
}
void RLink::Backward(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(0, coef[0] * d);
	(var->*func)(1, coef[1] * d);
	(var->*func)(2, coef[2] * d);
}

}
