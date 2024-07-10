#include <sbsolver.h>
#include <sbmessage.h>

#define USE_MKL

#if defined USE_MKL
# ifdef _WIN32
#  include <mkl_lapacke.h>
# else
//#  include <lapacke.h>
#  include <mkl_lapacke.h>
# endif
#endif

namespace Scenebuilder{;

static const real_t inf = numeric_limits<real_t>::max();
static const real_t eps = 1.0e-10;

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::Param::Param(){
	verbose         = false;
	methodMajor     = Solver::Method::Major::GaussNewton;
	methodMinor     = Solver::Method::Minor::GaussSeidel;
	methodLapack    = Solver::Method::Lapack::DGELS;
	minStepSize     =  0.01;
	maxStepSize     = 10.0;
	cutoffStepSize  =  0.001;
	hastyStepSize   = false;
	regularization  =  0.001;
	stateRegularization = 0.0;
    complRelaxation =  1.0;
	useHessian      = false;
	parallelize     = false;
	fixInitialState = false;
	fixInitialInput = false;
	parallelize     = false;
	enableSparse    = false;

	numIter.resize(10, 1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::Status::Status(){
	obj       = inf;
	objDiff   = inf;
	stepSize  = 0.0;
	iterCount = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::VariableInfo::VariableInfo(){
	num       = 0;
	numLocked = 0;
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
	RemoveFromArray(vars, var);
	ready = false;
}

void Solver::AddCon(Constraint* con){
	con->solver = this;
	cons.push_back(con);
	ready = false;
}

void Solver::DeleteCon(Constraint* con){
	RemoveFromArray(cons, con);
	ready = false;
}

void Solver::SetPriority(ID mask, uint level){
	Request req;
	req.type  = Request::Type::SetPriority;
	req.mask  = mask;
	req.level = level;
	requests.push_back(req);
	ready = false;
}

void Solver::Enable(ID mask, bool enable){
	Request req;
	req.type   = Request::Type::Enable;
	req.mask   = mask;
	req.enable = enable;
	requests.push_back(req);
	ready = false;
}

void Solver::Lock(ID mask, bool lock){
	Request req;
	req.type   = Request::Type::Lock;
	req.mask   = mask;
	req.lock   = lock;
	requests.push_back(req);
	ready = false;
}

void Solver::SetCorrection(ID mask, real_t rate, real_t lim){
	Request req;
	req.type   = Request::Type::SetCorrection;
	req.mask   = mask;
	req.rate   = rate;
	req.lim    = lim;
	requests.push_back(req);
	ready = false;
}

void Solver::SetConstraintWeight(ID mask, real_t weight){
	SetConstraintWeight(mask, vec3_t(weight, weight, weight));
}

void Solver::SetConstraintWeight(ID mask, vec3_t weight){
	Request req;
	req.type   = Request::Type::SetConstraintWeight;
	req.mask   = mask;
	req.weight = weight;
	requests.push_back(req);
	ready = false;
}

void Solver::SetVariableWeight(ID mask, real_t weight){
	SetVariableWeight(mask, vec3_t(weight, weight, weight));
}

void Solver::SetVariableWeight(ID mask, vec3_t weight){
	Request req;
	req.type   = Request::Type::SetVariableWeight;
	req.mask   = mask;
	req.weight = weight;
	requests.push_back(req);
	ready = false;
}

real_t Solver::CalcError(ID mask, bool sum_or_max){
	real_t E = 0.0;
	for(auto& con : cons){
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

	status = Status();

	ClearDDP();
}

void Solver::Reset(){
	for(auto& var : vars)
		var->Reset();
}

void Solver::Init(){
	// リクエスト処理
	for(Request& req : requests){
		for(auto& var : vars){
			if(!req.mask.Match(var))
				continue;
			if(req.type == Request::Type::Lock){
				var->locked = req.lock;
			}
			if(req.type == Request::Type::SetVariableWeight){
				var->weight = req.weight;
			}
		}
		for(auto& con : cons){
			if(!req.mask.Match(con))
				continue;
			if(req.type == Request::Type::Enable){
				con->enabled = req.enable;
			}
			if(req.type == Request::Type::SetPriority){
				con->level = req.level;
			}
			if(req.type == Request::Type::SetCorrection){
				con->corrRate = req.rate;
				con->corrMax  = req.lim;
			}
			if(req.type == Request::Type::SetConstraintWeight){
				con->weight = req.weight;
			}
		}
	}

	// 優先度の最大値を求める
	int maxVarType  = 0;
	int maxConType  = 0;
	int maxConLevel = 0;
	for(auto& var : vars){
		maxVarType = std::max(maxVarType, var->tag);
	}
	for(auto& con : cons){
		maxConType  = std::max(maxConType , con->tag  );
		maxConLevel = std::max(maxConLevel, con->level);
	}
	varInfoType .resize(maxVarType  + 1);
	conInfoType .resize(maxConType  + 1);
	conInfoLevel.resize(maxConLevel + 1);
	cons_level  .resize(maxConLevel + 1);

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

	if( param.methodMajor == Method::Major::DDP || 
		param.methodMajor == Method::Major::DDPContinuous )
		InitDDP();

	ready = true;
}

real_t Solver::CalcUpdatedObjective(real_t alpha){
	// temporarily update variables
	ModifyVariables(alpha);
	
	// return inf if variable update exceeds limit (so that this stepsize will not be chosen)
	real_t a2 = alpha*alpha;
	for(Variable* var : vars_unlocked){
		real_t d2 = var->scale2*var->dx.square();
		if(a2*d2 > var->dmax2)
			return inf;
	}

	for(auto& con : cons){
		if(!con->enabled)
			continue;
		
		con->CalcCoef ();
		con->CalcError();
	}

	return CalcObjective();
}

real_t Solver::CalcObjective(){
	// ddp has its own way to calculate cost
	if( param.methodMajor == Method::Major::DDP || 
		param.methodMajor == Method::Major::DDPContinuous ){
		return CalcObjectiveDDP();
	}

	real_t obj = 0.0;
	for(auto& con : cons){
		if(!con->enabled)
			continue;
		if(!con->active)
			continue;

		for(int k = 0; k < con->nelem; k++)
			obj += con->e[k];
	}
	
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
	// returning average step size may cause drifting behavior in infeasible problems
	// returning lower value is safer
	//return 0.5 * (a[0] + a[2]);
	return a[0];
}

void Solver::Prepare(){
	timer2.CountUS();

	vars_unlocked.clear();
	for(auto& var : vars){
		if(!var->locked)
			vars_unlocked.push_back(var);
	}

	for(Variable* var : vars_unlocked)
		var->ResetState();
	
	for(int i = 0; i < (int)varInfoType .size(); i++) varInfoType [i] = VariableInfo();
	for(int i = 0; i < (int)conInfoType .size(); i++) conInfoType [i] = ConstraintInfo();
	for(int i = 0; i < (int)conInfoLevel.size(); i++) conInfoLevel[i] = ConstraintInfo();

	for(auto& var : vars){
		varInfoType[var->tag].num++;
		if(var->locked)
			varInfoType[var->tag].numLocked++;
	}

	int T1 = timer2.CountUS();

	cons_active.clear();
	for(int i = 0; i < (int)cons_level.size(); i++)
		cons_level[i].clear();
	for(auto& con : cons){
		conInfoType [con->tag  ].num++;
		conInfoLevel[con->level].num++;

		if(con->enabled){
			conInfoType [con->tag  ].numEnabled++;
			conInfoLevel[con->level].numEnabled++;
		}
		else continue;

		con->CalcCoef   ();
		con->CalcHessian();
		con->CalcError  ();

		if(con->active){
            real_t esum = 0.0;
            for(int k = 0; k < con->nelem; k++)
                esum += con->e[k];
            
            conInfoType [con->tag  ].numActive++;
			conInfoLevel[con->level].numActive++;
			conInfoType [con->tag  ].error += esum;
			conInfoLevel[con->level].error += esum;

			cons_active.push_back(con);
			cons_level[con->level].push_back(con);
		}
	}

	int T2 = timer2.CountUS();

	if( param.methodMajor == Method::Major::GaussNewton ||
		param.methodMinor == Method::Minor::GaussSeidel){
		// links pointing to active constraints and unlocked variables
		for(auto& con : cons_active){
			con->links_active.clear();
			for(auto& link : con->links){
				if(!link->var->locked)
					con->links_active.push_back(link);
			}
		}
		for(auto& var : vars_unlocked){
			var->links_active.clear();
			for(auto& link : var->links){
				if(link->con->active)
					var->links_active.push_back(link);
			}
		}
	}
	int T3 = timer2.CountUS();
	//DSTR << "T1:  " << T1 << " T2: " << T2 << " T3: " << T3 << endl;
}

void Solver::CalcEquation(){
	dimvar          = 0;
	dimvar_weighted = 0;
	dimcon          = 0;
	
	for(auto& var : vars_unlocked){
		var->index = dimvar;
		dimvar += var->nelem;
		
		bool weighted = false;
		for(int j = 0; j < var->nelem; j++)
			weighted |= (var->weight[j] != 0.0);
		
		if(weighted){
			var->index_weighted = dimvar_weighted;
			dimvar_weighted += var->nelem;
		}
	}
	for(auto& con : cons_active){
		con->index = dimcon;
		dimcon += con->nelem;
	}
	
	// 変数あるいは拘束の数が不正
	if(dimvar == 0 || dimcon == 0)
		return;
		
	A.Allocate(dimcon + dimvar_weighted, dimvar);
	b.Allocate(dimcon + dimvar_weighted);
	
	mat_clear(A);
	vec_clear(b);
	
	pivot.resize(dimcon);

	//for(auto& con : cons_active){
#pragma omp parallel for if(param.parallelize)
	for(int i = 0; i < cons_active.size(); i++){
		Constraint* con = cons_active[i];
        vec3_t w = con->weight;
        
        // update multiplier of barrier inequality constraints
        /*for(Constraint* con : cons_active){
            if(con->type == Constraint::Type::InequalityBarrier){
                for(int k = 0; k < con->nelem; k++){
                    con->lambda[k] = param.complRelaxation/con->y[k];
                }
            }
        }*/
		/*
        // extra weight for barrier inequality constraints
        if(con->type == Constraint::Type::InequalityBarrier){
            for(int k = 0; k < con->nelem; k++){
                w[k] *= std::min(10.0, sqrt(param.complRelaxation)/std::max(eps, con->y[k]));
                //w[k] *= sqrt(con->lambda[k]/std::max(eps, con->y[k]));
            }
        }
		*/
		for(auto& link : con->links_active){
			link->RegisterCoef(A.SubMatrix(link->con->index, link->var->index, link->con->nelem, link->var->nelem), w);
		}
		con->RegisterCorrection(b.SubVector(con->index, con->nelem), con->weight);
		//con->RegisterDeviation (yvec, con->index);
	}


#pragma omp parallel for if(param.parallelize)
	//for(auto& var : vars_unlocked){
	for(int j = 0; j < vars_unlocked.size(); j++){
		Variable* var = vars_unlocked[j];
		if(var->index_weighted != -1){
			for(int j = 0; j < var->nelem; j++)
				A(dimcon + var->index_weighted + j, var->index + j) = var->weight[j];
		}
	}
}

void Solver::CalcDirection(){
	if(param.methodMajor == Method::Major::SteepestDescent){
		for(auto& var : vars_unlocked){
			var->ResetState();
		}
		for(auto& con : cons_active){
			for(int k = 0; k < con->nelem; k++)
				con->UpdateGradient(k);
		}	
	}
	if(param.methodMajor == Method::Major::GaussNewton){
		for(auto& con : cons_active)
			con->CalcCorrection();

		if(param.methodMinor == Method::Minor::Direct){
			timer2.CountUS();
			CalcEquation();
			int t2 = timer2.CountUS();

			timer2.CountUS();
			dxvec.Allocate(dimvar);
			vec_clear(dxvec);

			if(dimcon > 0){
	#if defined USE_MKL
				int nb = std::max(dimcon + dimvar_weighted, dimvar);
				b2.Allocate(nb);
				vec_clear(b2);

				for(int i = 0; i < dimcon; i++)
					b2(i) = b(i);
			
				bool   tryDposv = false;
				real_t dposvEps = 0.0;

				if(param.methodLapack == Method::Lapack::DGELS){
					// dgels
					//DSTR << "dimcon: " << dimcon << " dimvar: " << dimvar << " dimvar_weighted: " << dimvar_weighted << endl;
					int info = LAPACKE_dgels(LAPACK_COL_MAJOR, 'N', dimcon+dimvar_weighted, dimvar, 1, &A(0, 0), dimcon+dimvar_weighted, &b2(0), nb);
					if(info < 0){
						Message::Error("dgels: %d-th argument illegal", -info);
						Message::Error(" 6-th argument: %lx", &A(0, 0)             );
						Message::Error(" 7-th argument: %d", dimcon+dimvar_weighted);
						Message::Error(" 8-th argument: %lx", &b2(0)               );
						Message::Error(" 9-th argument: %d" , nb                   );
					}
					if(info > 0){
						Message::Error("dgels: matrix not full-rank");
						// try dposv with damping term
						tryDposv = true;
						dposvEps = 1.0e-5;
					}
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
				}
				if(param.methodLapack == Method::Lapack::DPOSV || tryDposv){
					// dposv
					AtrA.Allocate(A.n, A.n);
					mattr_mat_mul(A, A, AtrA, 1.0, 0.0);
					for(int i = 0; i < AtrA.m; i++)
						AtrA(i, i) += dposvEps;

					mattr_vec_mul(A, b, b2, 1.0, 0.0);
					LAPACKE_dposv(LAPACK_COL_MAJOR, 'U', dimcon, 1, &AtrA(0, 0), dimcon, &b2(0), dimcon);
				}

				for(int i = 0; i < dimvar; i++)
					dxvec(i) = b2(i);
	#else
	# error  USE_MKL is required
				//AtrA = A.trans()*A;
				//Atrb = A.trans()*b;
				//dxvec = AtrA.inv()*Atrb;
	#endif

				// check
				//vvec_t test = A*dxvec - b;
			}
			int t3 = timer2.CountUS();
			
			for(auto& var : vars_unlocked){
				var->RegisterDelta(dxvec);
			}

			//DSTR << "t1 t2 t3 " << t1 << " " << t2 << " " << t3 << endl;
		}
		else{
			for(auto& var : vars_unlocked) var->Prepare   ();
			for(auto& var : vars_unlocked) var->ResetState();
			for(auto& con : cons_active  ) con->ResetState();

			for(int n = 0; n < param.numIter[0]; n++){
				for(auto& var : vars){
					for(int k = 0; k < var->nelem; k++)
						var->UpdateVar3(k);
				}

				if(param.methodMinor == Method::Minor::Jacobi){
					for(auto& var : vars){
						var->dz.clear();
					}
					for(auto& con : cons_active){
						for(int k = 0; k < con->nelem; k++)
							con->UpdateConjugate(k);
					}

				}
			}
		}
	}
	if(param.methodMajor == Method::Major::Prioritized){
		for(auto& con : cons_active) con->CalcCorrection();
		for(auto& con : cons_active) con->ResetState();

		for(int L = (int)conInfoLevel.size()-1; L >= 0; L--){
			timer2.CountUS();
	
			for(int n = 1; n <= param.numIter[L]; n++){
				for(int l = 0; l <= L; l++){
					for(auto& con : cons_level[l]){
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
	if( param.methodMajor == Method::Major::DDP || 
		param.methodMajor == Method::Major::DDPContinuous ){
		for(auto& con : cons_active)
			con->CalcCorrection();

		CalcDirectionDDP();
	}
}

void Solver::ModifyVariables(real_t alpha){
	if( param.methodMajor == Method::Major::DDP || 
		param.methodMajor == Method::Major::DDPContinuous ){
		ModifyVariablesDDP(alpha);
		for(auto& var : vars_unlocked)
			var->Modify(1.0);
	}
	else{
		for(auto& var : vars_unlocked)
			var->Modify(alpha);
	}

}

void Solver::Step(){
	if(!ready)
		Init();

	timer.CountUS();
	Prepare();
	real_t objPrev = status.obj;
	status.obj      = CalcObjective();
	status.objDiff  = status.obj - objPrev;
	status.timePre = timer.CountUS();

	// 更新方向を計算
	timer.CountUS();
	CalcDirection();
	status.timeDir = timer.CountUS();

	// 直線探索で更新幅を計算
	timer.CountUS();
	status.stepSize = CalcStepSize();
	status.timeStep = timer.CountUS();

	timer.CountUS();
	// 変数を更新
	ModifyVariables(status.stepSize);
   	status.timeMod = timer.CountUS();

	if(param.verbose){
		Message::Out("iter:%d, step:%f, obj:%f", status.iterCount, status.stepSize, status.obj);
		DSTR << "iter:"    << status.iterCount
			 << " step:"   << status.stepSize
			 << " obj:"    << status.obj
			 << " tpre:"   << status.timePre
			 << " tdir:"   << status.timeDir
			 << " tstep:"  << status.timeStep
			 << " ttran:"  << status.timeTrans
			 << " tcost:"  << status.timeCost
			 << " tcostgrad: " << status.timeCostGrad
			 << " tback: " << status.timeBack
			 << " tmod:"   << status.timeMod
			 << endl;
	}
	status.iterCount++;

}

}
