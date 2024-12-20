﻿#include <sbvariable.h>
#include <sbsolver.h>

namespace Scenebuilder{;

Variable::Variable(uint _type, Solver* solver, ID _id, real_t _scale):ID(_id){
	solver->AddVar(this);
	SetScale(_scale);

	type = _type;
	switch(type){
	case Scalar:	nelem = 1; break;
	case Vec2:      nelem = 2; break;
	case Vec3:		nelem = 3; break;
	case Quat:
	default:		nelem = 3; break;
	}

	index          = 0;
	dmax           = 1.0;
	weight         = vec3_t(0.0, 0.0, 0.0);
	index_weighted = -1;
	locked         = false;
}

void Variable::Lock(bool on){
	locked = on;
}

void Variable::SetScale(real_t sc){
	scale      = sc;
	scale2     = sc*sc;
	scale_inv  = (real_t)1.0/sc;
	scale_inv2 = scale_inv*scale_inv;
}

void Variable::ResetState(){
	dx .clear();
	dz .clear();
	dzd.clear();
	dmax2 = dmax*dmax;
}

void Variable::Prepare(){
	if(locked)
		return;

	// ガウスザイデルで用いるJ^T*Jの対角成分 = Jの各列の二乗和を計算
	const real_t eps = (real_t)1.0e-10;
	
	J.clear();
	for(Link* link : links_active){
		link->AddColSqr(J);
	}
	
	// 対角成分の逆数
	for(int k = 0; k < nelem; k++){
		Jinv[k] = (1.0/(J[k] + eps));//(J[k] > eps ? (real_t)1.0/J[k] : (real_t)0.0);
	}
}

void Variable::UpdateVar1(uint k, real_t ddx){
	// no update if locked
	if(locked)
		return;

	// note that scaling ratio is multiplied twice
	dx[k] += ddx;
	
	// error update: 
	//for(Links::iterator it = links.begin(); it != links.end(); it++){
	//	Link* lnk = *it;
	//	lnk->Forward(k, ddx, &Constraint::UpdateError1);
	//}
}

void Variable::UpdateVar2(uint k, real_t ddx){
	// no update if locked
	if(locked)
		return;

	// note that scaling ratio is multiplied twice
	dx[k] += (1.0/weight[k]) * ddx;
}

void Variable::UpdateVar3(uint k){
	// update multiplier
	real_t ddx = (real_t)-1.0 * Jinv[k] * (dz[k] - dzd[k]);
	dx [k] = dx[k] + ddx;

	// update variable
	if(solver->param.methodMinor == Solver::Method::Minor::GaussSeidel){
		for(Link* link : links_active){
			link->Col(k, ddx, &Constraint::UpdateError3);
		}
	}
	else{
		for(Link* link : links_active){
			link->Col(k, ddx, &Constraint::UpdateError1);
		}
	}
}

void Variable::UpdateError(uint k){
	for(Link* link : links_active){
		link->Col(k, dx[k], &Constraint::UpdateError1);
	}
}

void Variable::UpdateConjugate1(uint k, real_t ddz){
	dz[k] += ddz;
}
void Variable::UpdateConjugate2(uint k, real_t _dz){
	dz[k]  = _dz;
}
void Variable::UpdateConjugate3(uint k, real_t ddzd){
	dzd[k] += ddzd;
}

void Variable::RegisterDelta(const Vector& dxvec){
	for(int n = 0; n < nelem; n++)
		dx[n] = dxvec(index+n);
}

}
