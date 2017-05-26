#include <sbsolver.h>
#include <sbmessage.h>

#define USE_MKL

#if defined USE_MKL
# include <mkl_lapacke.h>
#endif

#include <Foundation/UTPreciseTimer.h>
static UTPreciseTimer timer;

namespace Scenebuilder{;

static const real_t inf = numeric_limits<real_t>::max();

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::Param::Param(){
	verbose        = false;
	methodMajor    = Solver::Method::Major::GaussNewton1;
	methodMinor    = Solver::Method::Minor::GaussSeidel;
	numIterMajor   = 10;
	numIterMinor   = 10;
	minStepSize    =  0.01;
	maxStepSize    = 10.0;
	cutoffStepSize =  0.001;
	hastyStepSize  = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::State::State(){
	obj       = inf;
	objDiff   = inf;
	stepSize  = 0.0;
	iterCount = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::Solver(){

}

void Solver::AddVar(Variable* var){
	var->solver = this;
	vars.push_back(var);
}

void Solver::DeleteVar(Variable* var){
	vars.erase(find(vars.begin(), vars.end(), var));
}

void Solver::AddCon(Constraint* con){
	con->solver = this;
	cons.push_back(con);
}

void Solver::DeleteCon(Constraint* con){
	cons.erase(find(cons.begin(), cons.end(), con));
}

void Solver::Clear(){
	vars  .clear();
	cons  .clear();
	links .clear();

	state = State();
}

void Solver::Init(){
	//state.obj     = CalcObjective();
	//state.objDiff = inf;
}

real_t Solver::CalcUpdatedObjective(real_t alpha){
	// 更新量が上限を超えている場合はinfを返す
	real_t a2 = alpha*alpha;
	for(uint j = 0; j < vars.size(); j++){
		real_t d2 = vars[j]->dx.square();
		if(a2*d2 > vars[j]->dmax2)
			return inf;
	}
	
	// 変数を仮に更新して評価値を計算
	for(uint i = 0; i < vars.size(); i++)
		vars[i]->Modify(alpha);

	return CalcObjective();
}

real_t Solver::CalcObjective(){
	real_t obj = 0.0;
	for(uint i = 0; i < cons.size(); i++){
		Constraint* con = cons[i];
		if(!con->enabled)
			continue;
		
		con->CalcCoef ();
		con->CalcError();
		
		if(!con->active)
			continue;
		
		obj += con->y.square();
	}

	obj *= 0.5;

	return obj;
}

real_t Solver::CalcStepSize(){
	real_t amin = param.minStepSize;
	real_t amax = param.maxStepSize;
	real_t a[3], aprobe;
	real_t c[3], cprobe;
	
	if(amax - amin < param.cutoffStepSize)
		return 0.5 * (amin + amax);

	a[0] = amin;
	c[0] = CalcUpdatedObjective(amin);
	a[1] = a[2] = amax;
	c[1] = c[2] = CalcUpdatedObjective(amax);
	if(param.hastyStepSize && c[2] <= c[0])
		return a[2];

	if(c[2] > c[0]){
		while( c[1] > c[0] && a[1] - a[0] > param.cutoffStepSize * (amax - amin) ){
			a[1] = a[0] + 0.5*(a[1] - a[0]);
			c[1] = CalcUpdatedObjective(a[1]);
		}
		if(param.hastyStepSize)
			return a[1];
	}

	while(a[2] - a[0] > param.cutoffStepSize){
		if(a[1] - a[0] > a[2] - a[1]){
			aprobe = 0.5 * (a[0] + a[1]);
			cprobe = CalcUpdatedObjective(aprobe);
			if(cprobe <= c[1]){
				a[2] = a[1];
				a[1] = aprobe;
				c[2] = c[1];
				c[1] = cprobe;
			}
			else{
				a[0] = aprobe;
				c[0] = cprobe;
			}
		}
		else{
			aprobe = 0.5 * (a[1] + a[2]);
			cprobe = CalcUpdatedObjective(aprobe);
			if(cprobe < c[1]){
				a[0] = a[1];
				a[1] = aprobe;
				c[0] = c[1];
				c[1] = cprobe;
			}
			else{
				a[2] = aprobe;
				c[2] = cprobe;
			}
		}
	}
	return 0.5 * (a[0] + a[2]);
}

void Solver::CalcDirection(){
	timer.CountUS();
	vars_unlocked.clear();
	cons_active  .clear();

	for(uint j = 0; j < vars.size(); j++){
		Variable* var = vars[j];
		if(!var->locked)
			vars_unlocked.push_back(var);
	}
	for(uint i = 0; i < cons.size(); i++){
		Constraint* con = cons[i];
		if(!con->enabled) continue;

		con->CalcCoef ();
		con->CalcError();

		if(con->active)
			cons_active.push_back(con);
	}
	int timeCoef = timer.CountUS();
	//DSTR << "tcoef " << timeCoef << endl;

	if(param.methodMajor == Method::Major::SteepestDescent){
		for(uint j = 0; j < vars_unlocked.size(); j++){
			vars[j]->ResetState();
		}
		for(uint i = 0; i < cons_active.size(); i++){
			Constraint* con = cons_active[i];
			for(uint k = 0; k < con->nelem; k++)
				con->UpdateGradient(k);
		}	
	}
	if(param.methodMajor == Method::Major::GaussNewton1){		
		if(param.methodMinor == Method::Minor::Direct){
			timer.CountUS();
			uint dimvar = 0;
			uint dimcon = 0;
			uint idxvar = 0;
			uint idxcon = 0;
			for(uint j = 0; j < vars_unlocked.size(); j++){
				Variable* var = vars_unlocked[j];
				var->index = dimvar;
				dimvar += var->nelem;
			}
			for(uint i = 0; i < cons_active.size(); i++){
				Constraint* con = cons_active[i];
				con->index = dimcon;
				dimcon += con->nelem;
			}
			int t1 = timer.CountUS();

			//DSTR << "dimcon " << dimcon << " dimvar " << dimvar << endl;
			
			timer.CountUS();
			J.resize(dimcon, dimvar);
			y.resize(dimcon);
			J.clear();
			y.clear();
			for(uint i = 0; i < cons_active.size(); i++){
				Constraint* con = cons_active[i];
				for(uint j = 0; j < con->links.size(); j++){
					Link* lnk = con->links[j];
					if(lnk->var->locked)
						continue;
					lnk->RegisterCoef(J);
				}
				cons_active[i]->RegisterDeviation(y);
			}
			int t2 = timer.CountUS();
			
			timer.CountUS();
#if defined USE_MKL
			size_t ny = std::max(dimcon, dimvar);
			y2.resize(ny);
			for(uint i = 0; i < dimcon; i++)
				y2[i] = y[i];
			
			// dgels
			LAPACKE_dgels(LAPACK_COL_MAJOR, 'N', dimcon, dimvar, 1, &J[0][0], dimcon, &y2[0], ny);

			// dgelsd
			//vector<real_t> S;
			//S.resize(std::min(dimcon, dimvar));
			//real_t rcond = 0.01;
			//int    rank;
			//static vector<real_t> work;
			//static vector<int> iwork;
			//const int lwork  = 100000;
			//work.resize(lwork);
			//iwork.resize(lwork);
			//LAPACKE_dgelsd_work(LAPACK_COL_MAJOR, dimcon, dimvar, 1, &J[0][0], dimcon, &y2[0], ny,
			//	&S[0], rcond, &rank,
			//	&work[0], lwork, &iwork[0]);
			//DSTR << "rank " << rank << endl;
			
			// dgesv
			//vector<int> pivot;
			//pivot.resize(dimcon);
			//LAPACKE_dgesv(LAPACK_COL_MAJOR, dimcon, 1, &JtrJ[0][0], dimcon, &pivot[0], &y2[0], dimcon);

			// dposv
			//LAPACKE_dposv(LAPACK_COL_MAJOR, 'U', dimcon, 1, &JtrJ[0][0], dimcon, &y2[0], dimcon);
			
			dx.resize(dimvar);
			for(uint i = 0; i < dimvar; i++)
				dx[i] = -1.0 * y2[i];
#else
			JtrJ = J.trans()*J;
			Jtry = J.trans()*y;
			dx = -1.0 * (JtrJ.inv()*Jtry);
#endif
			int t3 = timer.CountUS();
			
			for(uint j = 0; j < vars_unlocked.size(); j++){
				Variable* var = vars_unlocked[j];
				var->RegisterDelta(dx);
			}

			//DSTR << "t1 t2 t3 " << t1 << " " << t2 << " " << t3 << endl;
		}
		else{
			for(uint j = 0; j < vars_unlocked.size(); j++) vars[j]->Prepare();
			for(uint j = 0; j < vars_unlocked.size(); j++) vars[j]->ResetState();
			for(uint i = 0; i < cons_active  .size(); i++) cons_active[i]->CalcCorrection();
			for(uint i = 0; i < cons_active  .size(); i++) cons_active[i]->ResetState();

			for(int n = 0; n < param.numIterMinor; n++){
				for(uint j = 0; j < vars.size(); j++){
					Variable* var = vars[j];
					for(uint k = 0; k < var->nelem; k++)
						var->UpdateVar3(k);
				}

				if(param.methodMinor == Method::Minor::Jacobi){
					for(uint j = 0; j < vars.size(); j++){
						vars[j]->dz.clear();
					}

					for(uint i = 0; i < cons_active.size(); i++){
						Constraint* con = cons_active[i];
						for(uint k = 0; k < con->nelem; k++)
							con->UpdateConjugate(k);
					}

				}
			}
		}
	}
	if(param.methodMajor == Method::Major::GaussNewton2){
		for(uint j = 0; j < vars.size(); j++){
			vars[j]->ResetState();
		}
		for(uint i = 0; i < cons_active.size(); i++){
			cons_active[i]->CalcCorrection();
		}
		for(uint i = 0; i < cons_active.size(); i++){
			cons_active[i]->ResetState();
		}
		for(int n = 0; n < param.numIterMinor; n++){
			for(uint i = 0; i < cons_active.size(); i++){
				Constraint* con = cons_active[i];
				for(uint k = 0; k < con->nelem; k++)
					con->UpdateMultiplier(k);
			}
			if(param.methodMinor == Method::Minor::Jacobi){
				for(uint i = 0; i < cons_active.size(); i++){
					cons_active[i]->dy.clear();
				}
				for(uint j = 0; j < vars.size(); j++){
					Variable* var = vars[j];
					for(uint k = 0; k < var->nelem; k++)
						var->UpdateError(k);
				}
			}
		}
	}
}

void Solver::Step(){
	// 更新方向を計算
	timer.CountUS();
	for(uint j = 0; j < vars.size(); j++)
		vars[j]->ResetState();
	CalcDirection();
	state.timeDir = timer.CountUS();

	// 直線探索で更新幅を計算
	timer.CountUS();
	state.stepSize = CalcStepSize();
	state.timeStep = timer.CountUS();

	//DSTR << " tdir "  << state.timeDir
	//	 << " tstep " << state.timeStep << endl;

	// 変数を更新
	for(uint j = 0; j < vars.size(); j++){
		if( !vars[j]->locked )
			vars[j]->Modify(state.stepSize);
	}

	real_t objPrev = state.obj;
	state.obj      = CalcObjective();
	state.objDiff  = state.obj - objPrev;
	if(param.verbose)
		Message::Out("iter:%d, step:%f, obj:%f", state.iterCount, state.stepSize, state.obj);
	state.iterCount++;
}

void Solver::Solve(){
	for(int n = 0; n < param.numIterMajor; n++)
		Step();
}

}
