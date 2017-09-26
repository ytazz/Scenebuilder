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
	minStepSize    =  0.01;
	maxStepSize    = 10.0;
	cutoffStepSize =  0.001;
	hastyStepSize  = false;

	numIter.resize(10, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::State::State(){
	obj       = inf;
	objDiff   = inf;
	stepSize  = 0.0;
	iterCount = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::ConstraintInfo::ConstraintInfo(){
	num        = 0;
	numEnabled = 0;
	numActive  = 0;
	error      = 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::Solver(){
	ready = false;
}

void Solver::AddVar(Variable* var){
	var->solver = this;
	vars.push_back(var);
	ready = false;
}

void Solver::DeleteVar(Variable* var){
	vars.erase(find(vars.begin(), vars.end(), var));
	ready = false;
}

void Solver::AddCon(Constraint* con){
	con->solver = this;
	cons.push_back(con);
	ready = false;
}

void Solver::DeleteCon(Constraint* con){
	cons.erase(find(cons.begin(), cons.end(), con));
	ready = false;
}

void Solver::SetPriority(ID mask, uint level){
	//int match = 0;
	//for(uint i = 0; i < cons.size(); i++){
	//	Constraint* con = cons[i];
	//	if(mask.Match(con)){
	//		con->SetPriority(level);
	//		match++;
	//	}
	//}
	//if(match)
	//	solver->ready = false;
	//return match;
	Request req;
	req.type  = Request::Type::SetPriority;
	req.mask  = mask;
	req.level = level;
	requests.push_back(req);
	ready = false;
}

void Solver::Enable(ID mask, bool enable){
	//int match = 0;
	//for(uint i = 0; i < cons.size(); i++){
	//	Constraint* con = cons[i];
	//	if(mask.Match(con)){
	//		con->enabled = enable;
	//		match++;
	//	}
	//}
	//return match;
	Request req;
	req.type   = Request::Type::Enable;
	req.mask   = mask;
	req.enable = enable;
	requests.push_back(req);
	ready = false;
}

void Solver::Lock(ID mask, bool lock){
	//int match = 0;
	//for(uint i = 0; i < vars.size(); i++){
	//	Variable* var = vars[i];
	//	if(mask.Match(var)){
	//		var->Lock(lock);
	//		match++;
	//	}
	//}
	//return match;
	Request req;
	req.type   = Request::Type::Lock;
	req.mask   = mask;
	req.lock   = lock;
	requests.push_back(req);
	ready = false;
}

void Solver::SetCorrectionRate(ID mask, real_t rate, real_t lim){
	//int match = 0;
	//for(uint i = 0; i < cons.size(); i++){
	//	Constraint* con = cons[i];
	//	if(mask.Match(con)){
	//		con->corrRate = rate;
	//		con->corrMax  = lim;
	//		match++;
	//	}
	//}
	//return match;
	Request req;
	req.type   = Request::Type::SetCorrectionRate;
	req.mask   = mask;
	req.rate   = rate;
	req.lim    = lim;
	requests.push_back(req);
	ready = false;
}

real_t Solver::CalcError(ID mask, bool sum_or_max){
	real_t E = 0.0;
	for(uint i = 0; i < cons.size(); i++){
		Constraint* con = cons[i];
		if(mask.Match(con) && con->enabled && con->active){
			for(int k = 0; k < con->nelem; k++){
				if(sum_or_max)
					 E += con->e[k];
				else E = std::max(E, con->e[k]);
			}
		}
	}
	return E;
}

void Solver::Clear(){
	vars    .clear();
	cons    .clear();
	links   .clear();

	state = State();
}

void Solver::Reset(){
	for(uint i = 0; i < vars.size(); i++)
		vars[i]->Reset();
}

void Solver::Init(){
	//state.obj     = CalcObjective();
	//state.objDiff = inf;

	// リクエスト処理
	for(uint i = 0; i < (uint)requests.size(); i++){
		Request& req = requests[i];
		for(uint j = 0; j < (uint)vars.size(); j++){
			Variable* var = vars[j];
			if(!req.mask.Match(var))
				continue;
			if(req.type == Request::Type::Lock)
				var->locked = req.lock;
		}
		for(uint j = 0; j < (uint)cons.size(); j++){
			Constraint* con = cons[j];
			if(!req.mask.Match(con))
				continue;
			if(req.type == Request::Type::Enable){
				con->enabled = req.enable;
			}
			if(req.type == Request::Type::SetPriority){
				con->level = req.level;
			}
			if(req.type == Request::Type::SetCorrectionRate){
				con->corrRate = req.rate;
				con->corrMax  = req.lim;
			}
		}
	}

	// 優先度の最大値を求める
	int maxLevel = 0;
	int maxType  = 0;
	for(uint i = 0; i < cons.size(); i++){
		maxLevel = std::max(maxLevel, cons[i]->level);
		maxType  = std::max(maxType , cons[i]->type );
	}

	infoType  .resize(maxType  + 1);
	infoLevel .resize(maxLevel + 1);
	cons_level.resize(maxLevel + 1);

	// レベル数とmaxIterのサイズが合わせてデフォルト値をセット
	//uint sz = (uint)maxIter.size();
	//uint sznew = maxLevel+1;
	//if(sz < sznew){
	//	maxIter.resize(sznew);
	//	for(uint i = sz; i < sznew; i++) 
	//		maxIter[i] = maxIterDefault;
	//}

	// ログ有効時
	//int i = Logging::MajorLoop;
	//if(doLog[i]){
	//	if(!file[i].is_open()){
	//		file[i].open("log_major.csv");
	//		LogLabel(i);
	//	}
	//	LogValue(i);
	//}
	//else{
	//	if(file[i].is_open())
	//		file[i].close();
	//}

	ready = true;
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
			a[1] = a[0] + 0.5 * (a[1] - a[0]);
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
	for(uint i = 0; i < (uint)cons_level.size(); i++)
		cons_level[i].clear();

	for(uint j = 0; j < vars.size(); j++){
		Variable* var = vars[j];
		if(!var->locked)
			vars_unlocked.push_back(var);
	}

	for(uint i = 0; i < (uint)infoType .size(); i++) infoType [i] = ConstraintInfo();
	for(uint i = 0; i < (uint)infoLevel.size(); i++) infoLevel[i] = ConstraintInfo();

	for(uint i = 0; i < cons.size(); i++){
		Constraint* con = cons[i];
		infoType [con->type ].num++;
		infoLevel[con->level].num++;

		if(con->enabled){
			 infoType [con->type ].numEnabled++;
			 infoLevel[con->level].numEnabled++;
		}
		else continue;

		con->CalcCoef ();
		con->CalcError();

		if(con->active){
			infoType [con->type ].numActive++;
			infoType [con->type ].error += con->y.norm();
			
			infoLevel[con->level].numActive++;
			infoLevel[con->level].error += con->y.norm();

			cons_active.push_back(con);
			cons_level[con->level].push_back(con);
		}
	}
	int timeCoef = timer.CountUS();
	//DSTR << "tcoef " << timeCoef << endl;

	if(param.methodMajor == Method::Major::SteepestDescent){
		for(uint j = 0; j < vars_unlocked.size(); j++){
			vars[j]->ResetState();
		}
		for(uint i = 0; i < cons_active.size(); i++){
			Constraint* con = cons_active[i];
			for(int k = 0; k < con->nelem; k++)
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

			// 変数あるいは拘束の数が不正
			if(dimvar == 0 || dimcon == 0)
				return;
			
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
			LAPACKE_dgels(LAPACK_COL_MAJOR, 'N', (int)dimcon, (int)dimvar, 1, &J[0][0], (int)dimcon, &y2[0], (int)ny);

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

			for(int n = 0; n < param.numIter[0]; n++){
				for(uint j = 0; j < vars.size(); j++){
					Variable* var = vars[j];
					for(int k = 0; k < var->nelem; k++)
						var->UpdateVar3(k);
				}

				if(param.methodMinor == Method::Minor::Jacobi){
					for(uint j = 0; j < vars.size(); j++){
						vars[j]->dz.clear();
					}

					for(uint i = 0; i < cons_active.size(); i++){
						Constraint* con = cons_active[i];
						for(int k = 0; k < con->nelem; k++)
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
		for(int n = 0; n < param.numIter[0]; n++){
			for(uint i = 0; i < cons_active.size(); i++){
				Constraint* con = cons_active[i];
				for(int k = 0; k < con->nelem; k++)
					con->UpdateMultiplier(k);
			}
			if(param.methodMinor == Method::Minor::Jacobi){
				for(uint i = 0; i < cons_active.size(); i++){
					cons_active[i]->dy.clear();
				}
				for(uint j = 0; j < vars.size(); j++){
					Variable* var = vars[j];
					for(int k = 0; k < var->nelem; k++)
						var->UpdateError(k);
				}
			}
		}
	}
	if(param.methodMajor == Method::Major::Prioritized){
		for(int i = 0; i < (int)cons_active.size(); i++)
			cons_active[i]->CalcCorrection();

		for(int i = 0; i < (int)cons_active.size(); i++)
			cons_active[i]->ResetState();

		for(int L = (int)infoLevel.size()-1; L >= 0; L--){
			timer.CountUS();
	
			for(int n = 1; n <= param.numIter[L]; n++){
				for(int l = 0; l <= L; l++){
					for(int i = 0; i < (int)cons_level[l].size(); i++){
						Scenebuilder::Constraint* con = cons_level[l][i];
						for(int k = 0; k < con->nelem; k++)
							con->UpdateMultiplier(k);
					}
				}
			
				/*
				// ガウス-ザイデルの並列計算
				// 同一phase内（干渉しない拘束）は並列実行する
				for(int phase = 0; phase < (int)cons_arranged.size(); phase++){
					int ncon = (int)cons_arranged[phase].size();
					#pragma omp parallel for if(ncon > 10)
					for(int i = 0; i < ncon; i++){
						Constraint* con = cons_arranged[phase][i];
						if(!con->enabled)
							continue;
						if(!con->active)
							continue;
						if((int)con->level > l)
							continue;
						for(uint k = 0; k < con->nelem; k++)
							con->UpdateMultiplierCorr(k);
					}
				}
				*/
			}

			//DSTR << "level " << L << ": " << ptimer.CountUS() << endl;
		}
	}
}

void Solver::Step(){
	if(!ready)
		Init();

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

	// ログ有効時
	//int i = Logging::MajorLoop;
 	//if(doLog[i]){
	//	LogValue(i);
	//}
}

}
