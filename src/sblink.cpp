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
	for(int k = 0; k < con->nelem; k++)
		v[k] += coefsqr * var->scale2;
}

void SLink::AddColSqr(vec3_t& v){
	for(int k = 0; k < var->nelem; k++)
		v[k] += coefsqr * var->scale2;
}

void SLink::AddError(){
	if(con->nelem == 1){
		con->y[0] += coef * ((SVar *)var)->val;
	}
	else if(con->nelem == 2){
		con->y[0] += coef * ((V2Var*)var)->val[0];
		con->y[1] += coef * ((V2Var*)var)->val[1];
	}
	else{
		con->y += coef * ((V3Var*)var)->val;
	}
}

void SLink::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	for(int ii = 0; ii < con->nelem; ii++)
		J[i+ii][j+ii] = w[ii] * coef;
}	

void SLink::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(k, coef * d);
}

void SLink::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(k, coef * d[k]);
}

void SLink::ColTrans(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(k, coef * d);
}

//-------------------------------------------------------------------------------------------------

void V2Link::SetCoef(vec2_t k){
	coef = k;
	for(uint i = 0; i < 2; i++)
		coefsqr[i] = k[i]*k[i];
}

//-------------------------------------------------------------------------------------------------

void V3Link::SetCoef(vec3_t k){
	coef = k;
	for(uint i = 0; i < 3; i++)
		coefsqr[i] = k[i]*k[i];
}

//-------------------------------------------------------------------------------------------------
// c % x = [c[1] * x[2] - c[2] * x[1]
//          c[2] * x[0] - c[0] * x[2]
// 	        c[0] * x[1] - c[1] * x[0]]
// J = [ 0    -c[2]  c[1]
//       c[2]  0    -c[0]
// 	    -c[1]  c[0]  0   ]

void X3Link::AddRowSqr(vec3_t& v){
	v[0] += (coefsqr[2] + coefsqr[1]) * var->scale2;
	v[1] += (coefsqr[2] + coefsqr[0]) * var->scale2;
	v[2] += (coefsqr[1] + coefsqr[0]) * var->scale2;
}

void X3Link::AddColSqr(vec3_t& v){
	v[0] += (coefsqr[2] + coefsqr[1]) * var->scale2;
	v[1] += (coefsqr[2] + coefsqr[0]) * var->scale2;
	v[2] += (coefsqr[1] + coefsqr[0]) * var->scale2;
}

void X3Link::AddError(){
	con->y += coef % ((V3Var*)var)->val;
}

void X3Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	J[i+0][j+0] =  0.0           ; J[i+0][j+1] = -w[0] * coef[2]; J[i+0][j+2] =  w[0] * coef[1];
	J[i+1][j+0] =  w[1] * coef[2]; J[i+1][j+1] =  0.0           ; J[i+1][j+2] = -w[1] * coef[0];
	J[i+2][j+0] = -w[2] * coef[1]; J[i+2][j+1] =  w[2] * coef[0]; J[i+2][j+2] =  0.0           ;
}	

void X3Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
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

void X3Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	if(k == 0){
		(con->*func)(0, coef[1]*d[2] - coef[2]*d[1]);
	}
	else if(k == 1){
		(con->*func)(1, coef[2]*d[0] - coef[0]*d[2]);
	}
	else{
		(con->*func)(2, coef[0]*d[1] - coef[1]*d[0]);
	}
}

void X3Link::ColTrans(uint k, real_t d, Variable::UpdateFunc func){
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

void C2Link::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[1] += coefsqr[1] * var->scale2;
}

void C2Link::AddColSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[0] += coefsqr[1] * var->scale2;
}

void C2Link::AddError(){
	con->y[0] += coef[0] * ((SVar*)var)->val;
	con->y[1] += coef[1] * ((SVar*)var)->val;
}

void C2Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	J[i+0][j] = w[0] * coef[0];
	J[i+1][j] = w[1] * coef[1];
}	

void C2Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0] * d);
	(con->*func)(1, coef[1] * d);
}

void C2Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(k, coef[k] * d[0]);
}

void C2Link::ColTrans(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(0, coef[k] * d);
}

//-------------------------------------------------------------------------------------------------

void C3Link::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[1] += coefsqr[1] * var->scale2;
	v[2] += coefsqr[2] * var->scale2;
}

void C3Link::AddColSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[0] += coefsqr[1] * var->scale2;
	v[0] += coefsqr[2] * var->scale2;
}

void C3Link::AddError(){
	con->y += coef * ((SVar*)var)->val;
}

void C3Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	J[i+0][j] = w[0] * coef[0];
	J[i+1][j] = w[1] * coef[1];
	J[i+2][j] = w[2] * coef[2];
}	

void C3Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0] * d);
	(con->*func)(1, coef[1] * d);
	(con->*func)(2, coef[2] * d);
}

void C3Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(k, coef[k] * d[0]);
}

void C3Link::ColTrans(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(0, coef[k] * d);
}

//-------------------------------------------------------------------------------------------------

void R2Link::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[0] += coefsqr[1] * var->scale2;
}

void R2Link::AddColSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[1] += coefsqr[1] * var->scale2;
}

void R2Link::AddError(){
	con->y[0] += coef * ((V2Var*)var)->val;
}

void R2Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	J[i][j+0] = w[0] * coef[0];
	J[i][j+1] = w[0] * coef[1];
}	

void R2Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[k] * d);
}

void R2Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0] * d[0] + coef[1] * d[1]);
}

void R2Link::ColTrans(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(0, coef[0] * d);
	(var->*func)(1, coef[1] * d);
}

//-------------------------------------------------------------------------------------------------

void R3Link::AddRowSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[0] += coefsqr[1] * var->scale2;
	v[0] += coefsqr[2] * var->scale2;
}

void R3Link::AddColSqr(vec3_t& v){
	v[0] += coefsqr[0] * var->scale2;
	v[1] += coefsqr[1] * var->scale2;
	v[2] += coefsqr[2] * var->scale2;
}

void R3Link::AddError(){
	con->y[0] += coef * ((V3Var*)var)->val;
}

void R3Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	J[i][j+0] = w[0] * coef[0];
	J[i][j+1] = w[0] * coef[1];
	J[i][j+2] = w[0] * coef[2];
}	

void R3Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[k] * d);
}

void R3Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0] * d[0] + coef[1] * d[1] + coef[2] * d[2]);
}

void R3Link::ColTrans(uint k, real_t d, Variable::UpdateFunc func){
	(var->*func)(0, coef[0] * d);
	(var->*func)(1, coef[1] * d);
	(var->*func)(2, coef[2] * d);
}

//-------------------------------------------------------------------------------------------------

void M2Link::SetCoef(const mat2_t& m){
	coef = m;
	for(uint i = 0; i < 2; i++)for(uint j = 0; j < 2; j++)
		coefsqr[i][j] = coef[i][j]*coef[i][j];
}

void M2Link::AddColSqr(vec3_t& v){
	v[0] += (coefsqr[0][0] + coefsqr[1][0]) * var->scale2;
	v[1] += (coefsqr[0][1] + coefsqr[1][1]) * var->scale2;
}

void M2Link::AddRowSqr(vec3_t& v){
	v[0] += (coefsqr[0][0] + coefsqr[0][1]) * var->scale2;
	v[1] += (coefsqr[1][0] + coefsqr[1][1]) * var->scale2;
}

void M2Link::AddError(){
	con->y[0] += coef.row(0) * ((V2Var*)var)->val;
	con->y[1] += coef.row(1) * ((V2Var*)var)->val;
}

void M2Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	for(int ii = 0; ii < 2; ii++)for(int jj = 0; jj < 2; jj++)
		J[i+ii][j+jj] = w[ii] * coef[ii][jj];
}	

void M2Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0][k] * d);
	(con->*func)(1, coef[1][k] * d);
}

void M2Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(k, coef[k][0] * d[0] + coef[k][1] * d[1]);
}

void M2Link::ColTrans(uint k, real_t d, Variable  ::UpdateFunc func){
	(var->*func)(0, coef[k][0] * d);
	(var->*func)(1, coef[k][1] * d);
}

//vec2_t M2Link::Backward(vec2_t v){
//	return coef.trans() * v;
//}

//-------------------------------------------------------------------------------------------------

void M3Link::SetCoef(const mat3_t& m){
	coef = m;
	for(uint i = 0; i < 3; i++)for(uint j = 0; j < 3; j++)
		coefsqr[i][j] = coef[i][j]*coef[i][j];
}

void M3Link::AddColSqr(vec3_t& v){
	v[0] += (coefsqr[0][0] + coefsqr[1][0] + coefsqr[2][0]) * var->scale2;
	v[1] += (coefsqr[0][1] + coefsqr[1][1] + coefsqr[2][1]) * var->scale2;
	v[2] += (coefsqr[0][2] + coefsqr[1][2] + coefsqr[2][2]) * var->scale2;
}

void M3Link::AddRowSqr(vec3_t& v){
	v[0] += (coefsqr[0][0] + coefsqr[0][1] + coefsqr[0][2]) * var->scale2;
	v[1] += (coefsqr[1][0] + coefsqr[1][1] + coefsqr[1][2]) * var->scale2;
	v[2] += (coefsqr[2][0] + coefsqr[2][1] + coefsqr[2][2]) * var->scale2;
}

void M3Link::AddError(){
	con->y += coef * ((V3Var*)var)->val;
}

void M3Link::RegisterCoef(vmat_t& J, int i, int j, vec3_t w){
	//uint i = con->index;
	//uint j = var->index;
	for(int ii = 0; ii < 3; ii++)for(int jj = 0; jj < 3; jj++)
		J[i+ii][j+jj] = w[ii] * coef[ii][jj];
}	

void M3Link::Col(uint k, real_t d, Constraint::UpdateFunc func){
	(con->*func)(0, coef[0][k] * d);
	(con->*func)(1, coef[1][k] * d);
	(con->*func)(2, coef[2][k] * d);
}

void M3Link::Row(uint k, vec3_t& d, Constraint::UpdateFunc func){
	(con->*func)(k, coef[k][0] * d[0] + coef[k][1] * d[1] + coef[k][2] * d[2]);
}

void M3Link::ColTrans(uint k, real_t d, Variable  ::UpdateFunc func){
	(var->*func)(0, coef[k][0] * d);
	(var->*func)(1, coef[k][1] * d);
	(var->*func)(2, coef[k][2] * d);
}

//vec3_t M3Link::Backward(vec3_t v){
//	return coef.trans() * v;
//}

}
