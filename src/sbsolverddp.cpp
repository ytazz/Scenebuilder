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

#include <sbtimer.h>

namespace Scenebuilder{;

static Timer timer;
static Timer timer2;

static const real_t inf = numeric_limits<real_t>::max();

inline real_t square(real_t x){
	return x*x;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Solver::SubInput* Solver::Input::Find(Variable* var){
	for(SubInput* subin : subinput){
		if(subin->var == var)
			return subin;
	}
	return 0;
}

Solver::SubState* Solver::State::Find(Variable* var){
	for(SubState* subst : substate){
		if(subst->var == var)
			return subst;
	}
	return 0;
}

Solver::SubState* Solver::AddStateVar(Variable* var, int k){
	if(state.size() <= k)
		state.resize(k+1);

	if(!state[k])
		state[k] = new State();

	SubState* subst = new SubState();
	subst->var = var;
	state[k]->substate.push_back(subst);

    return subst;
}

Solver::SubInput* Solver::AddInputVar(Variable* var, int k){
	if(input.size() <= k)
		input.resize(k+1);

	if(!input[k])
		input[k] = new Input();

	SubInput* subin = new SubInput();
	subin->var = var;
	input[k]->subinput.push_back(subin);

    return subin;
}

Solver::SubTransition* Solver::AddTransitionCon(Constraint* con, int k){
	if(transition.size() <= k)
		transition.resize(k+1);

	if(!transition[k])
		transition[k] = new Transition();

	State* st_x0 = state[k+0];
	State* st_x1 = state[k+1];
	Input* in_u  = input[k+0];

	SubTransition* subtr = new SubTransition();
	subtr->con = con;
	for(Link* l : con->links){
		SubState* subst;
		SubInput* subin;

		// variable that belongs to st_x1 must be unique
		subst = st_x1->Find(l->var);
		if(subst){
			subtr->x1 = subst;
		}
		subst = st_x0->Find(l->var);
		if(subst){
			SubStateLink xl;
			xl.x    = subst;
			xl.link = l;
			subtr->x0.push_back(xl);
		}
		subin = in_u->Find(l->var);
		if(subin){
			SubInputLink ul;
			ul.u    = subin;
			ul.link = l;
			subtr->u.push_back(ul);
		}
	}

	transition[k]->subtran.push_back(subtr);

    return subtr;
}

Solver::SubCost* Solver::AddCostCon (Constraint* con, int k){
	if(cost.size() <= k)
		cost.resize(k+1);

	if(!cost[k])
		cost[k] = new Cost();

	State* st_x = (k < state.size() ? (State*)state[k] : (State*)0);
	Input* in_u = (k < input.size() ? (Input*)input[k] : (Input*)0);

	SubCost* subcost = new SubCost();
	subcost->con = con;

	for(Link* l : con->links){
		SubState* subst;
		SubInput* subin;

		subst = (st_x ? st_x->Find(l->var) : (SubState*)0);
		if(subst){
			SubStateLink xl;
			xl.x    = subst;
			xl.link = l;
			subcost->x.push_back(xl);
		}
		subin = (in_u ? in_u->Find(l->var) : (SubInput*)0);
		if(subin){
			SubInputLink ul;
			ul.u    = subin;
			ul.link = l;
			subcost->u.push_back(ul);
		}
	}

	cost[k]->subcost.push_back(subcost);

    return subcost;
}

void Solver::SetTimeStep(real_t _dt, int k){
	if(dt.size() <= k)
		dt.resize(k+1);

	dt[k] = _dt;
}

void Solver::InitDDP(){
	N = (int)state.size()-1;

	for(int k = 0; k < state.size(); k++){
        State* st = state[k];
		st->dim = 0;

		for(SubState* subst : st->substate){
			if(subst->var->locked)
				continue;

			subst->index = st->dim;
			st->dim += subst->var->nelem;
		}
	}

	for(int k = 0; k < input.size(); k++){
        Input* in = input[k];
		in->dim = 0;

		for(SubInput* subin : in->subinput){
			if(subin->var->locked)
				continue;

			subin->index = in->dim;
			in->dim += subin->var->nelem;
		}
	}

	for(int k = 0; k < N; k++){
		Transition* tr = transition[k];

		for(SubTransition* subtr : tr->subtran){
			if(!subtr->con->enabled)
				continue;
            if(subtr->x1->var->locked)
                continue;
			
			int n  = subtr->con->nelem;			
			subtr->b.resize(n);
			subtr->b.clear();
            
			for(SubStateLink& x0 : subtr->x0){
				if(x0.x->var->locked)
					continue;

				int m  = x0.x->var->nelem;
				x0.A.resize(n, m);
				x0.A.clear();
			}
            
            for(SubInputLink& u : subtr->u){
				if(u.u->var->locked)
					continue;

				int m = u.u->var->nelem;
				u.A.resize(n, m);
				u.A.clear();			
			}
		}
	}

	for(int k = 0; k <= N; k++){
		if(!cost[k])
			continue;

		for(SubCost* subcost : cost[k]->subcost){     
            if(!subcost->con->enabled)
				continue;
	
			int n  = subcost->con->nelem;
			subcost->b.resize(n);
			
			for(SubStateLink& x : subcost->x){
				if(x.x->var->locked)
					continue;

				int m = x.x->var->nelem;
				x.A.resize(n, m);
				x.A.clear();
			}

			if(k < N){
				for(SubInputLink& u : subcost->u){
					if(u.u->var->locked)
						continue;

					int m = u.u->var->nelem;
					u.A.resize(n, m);
					u.A.clear();
				}
			}            
		}
	}

	dx       .resize(N+1);
	du       .resize(N);
	fx       .resize(N);
	fu       .resize(N);
	fcor     .resize(N);
    L        .resize(N+1);
	Lx       .resize(N+1);
	Lxx      .resize(N+1);
	Lu       .resize(N);
	Luu      .resize(N);
	Lux      .resize(N);
	Q        .resize(N);
	Qx       .resize(N);
	Qu       .resize(N);
	Qxx      .resize(N);
	Quu      .resize(N);
	Quuinv   .resize(N);
	Quuinv_Qu.resize(N);
	Qux      .resize(N);
	V        .resize(N+1);
	Vx       .resize(N+1);
	Vxx      .resize(N+1);
	dV       .resize(N+1);
	dVx      .resize(N+1);
	dVxx     .resize(N+1);
	Vxx_fcor .resize(N);
	Vxx_fx   .resize(N);
	Vxx_fu   .resize(N);
	Vx_plus_Vxx_fcor.resize(N);
	Quuinv_Qux      .resize(N);
	Qu_plus_Qux_dx  .resize(N);

	for(int k = 0; k <= N; k++){
		int nx  = state[k]->dim;

		dx  [k].Allocate(nx);
		Lx  [k].Allocate(nx);
		Lxx [k].Allocate(nx, nx);
		Vx  [k].Allocate(nx);
		Vxx [k].Allocate(nx, nx);
		dVx [k].Allocate(nx);
		dVxx[k].Allocate(nx, nx);
		if(k == 0){
			Vxxinv.Allocate(nx, nx);
		}

		if(k < N){
			int nu  = input[k]->dim;
			int nx1 = state[k+1]->dim;

			du       [k].Allocate(nu      );
			fx       [k].Allocate(nx1, nx );
			fu       [k].Allocate(nx1, nu );
			fcor     [k].Allocate(nx1     );
            Lu       [k].Allocate(nu      );
			Luu      [k].Allocate(nu , nu );
			Lux      [k].Allocate(nu , nx );
			Qx       [k].Allocate(nx      );
			Qu       [k].Allocate(nu      );
			Qxx      [k].Allocate(nx , nx );
			Quu      [k].Allocate(nu , nu );
			Quuinv   [k].Allocate(nu , nu );
			Quuinv_Qu[k].Allocate(nu      );
			Qux      [k].Allocate(nu , nx );
	        Vxx_fcor [k].Allocate(nx1     );
	        Vxx_fx   [k].Allocate(nx1, nx );
	        Vxx_fu   [k].Allocate(nx1, nu );
			Vx_plus_Vxx_fcor[k].Allocate(nx1);
			Quuinv_Qux      [k].Allocate(nu, nx);
			Qu_plus_Qux_dx  [k].Allocate(nu);
		}
	}

}

void Solver::ClearDDP(){
	state     .clear();
	input     .clear();
	transition.clear();
	cost      .clear();
	
	N = 0;
	dx       .clear();
	du       .clear();
	fx       .clear();
	fu       .clear();
	fcor     .clear();
    L        .clear();
	Lx       .clear();
	Lxx      .clear();
	Lu       .clear();
	Luu      .clear();
	Lux      .clear();
	Q        .clear();
	Qx       .clear();
	Qu       .clear();
	Qxx      .clear();
	Quu      .clear();
	Qux      .clear();
	Quuinv   .clear();
	Quuinv_Qu.clear();
    V        .clear();
	Vx       .clear();
	Vxx      .clear();

}

void Solver::CalcTransitionDDP(){
	timer2.CountUS();

	vec3_t one(1.0, 1.0, 1.0);

#pragma omp parallel for
	for(int k = 0; k < N; k++){
		Transition* tr = transition[k];
		real_t h, hinv;
		if(param.methodMajor == Method::Major::DDPContinuous){
			h    = dt[k];
			hinv = 1.0/h;	
		}

		mat_clear(fx  [k]);
		mat_clear(fu  [k]);
        vec_clear(fcor[k]);

		for(SubTransition* subtr : tr->subtran){
			if(!subtr->con->enabled)
				continue;
			if(!subtr->con->active)
				continue;
            if(subtr->x1->var->locked)
                continue;
			
			int i0 = subtr->con->index;
			int n  = subtr->con->nelem;
			// for fcor, constraint weight should be ignored
            subtr->con->RegisterCorrection(subtr->b, one, 0);

			// set fx
			int ix0 = 0;
			for(SubStateLink& x0 : subtr->x0){
				if(x0.x->var->locked)
					continue;

				int m = x0.x->var->nelem;
				x0.link->RegisterCoef(x0.A, 0, 0, one);

				// continuous time
				if(param.methodMajor == Method::Major::DDPContinuous){
					if(ix0 != 0){
						for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
							fx[k](subtr->x1->index+i, x0.x->index+j) = -x0.A[i][j]*hinv;
					}
				}
				// discrete time
				else{
					for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
						fx[k](subtr->x1->index+i, x0.x->index+j) = -x0.A[i][j];
				}

				ix0++;
			}
            
            // set fu
			for(SubInputLink& u : subtr->u){
				if(u.u->var->locked)
					continue;

				//int j0 = u->var->index; 
				int m = u.u->var->nelem;
				u.link->RegisterCoef(u.A, 0, 0, one);

				// continuous time
				if(param.methodMajor == Method::Major::DDPContinuous){
					for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
						fu[k](subtr->x1->index+i, u.u->index+j) = -u.A[i][j]*hinv;
				}
				// discrete time
				else{
					for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
						fu[k](subtr->x1->index+i, u.u->index+j) = -u.A[i][j];
				}
			}
            
            // set correction term
			// continuous time
			if(param.methodMajor == Method::Major::DDPContinuous){
				for(int i = 0; i < n; i++)
					fcor[k](subtr->x1->index+i) = subtr->b[i]*hinv;				
			}
			// discrete time
			else{
				for(int i = 0; i < n; i++)
					fcor[k](subtr->x1->index+i) = subtr->b[i];
			}
		}
	}
	status.timeTrans = timer2.CountUS();
	//DSTR << " tf: " << tf << endl;
	/*
	FILE* file = fopen("fx.csv", "w");
	int k = 5;
	for(int i = 0; i < fx[k].m; i++){
		for(int j = 0; j < fx[k].n; j++){
			fprintf(file, "%f, ", fx[k](i,j));
		}
		fprintf(file, "\n");
	}
	fclose(file);

	file = fopen("fu.csv", "w");
	for(int i = 0; i < fu[k].m; i++){
		for(int j = 0; j < fu[k].n; j++){
			fprintf(file, "%f, ", fu[k](i,j));
		}
		fprintf(file, "\n");
	}
	fclose(file);
	*/
}

void Solver::CalcCostDDP(){
	for(int k = 0; k <= N; k++){
		L  [k] = 0.0;
	}

	timer2.CountUS();
	// calculate state cost
#pragma omp parallel for
	for(int k = 0; k <= N; k++){
		if(!cost[k])
			continue;

		for(SubCost* subcost : cost[k]->subcost){     
            if(!subcost->con->enabled)
				continue;
			if(!subcost->con->active)
				continue;

			int n  = subcost->con->nelem;
			subcost->con->RegisterDeviation(subcost->b, 0);
			//subcost->con->RegisterCorrection(subcost->b, subcost->con->weight, 0);

			// sum up L
			if( subcost->con->type == Constraint::Type::Equality ||
				subcost->con->type == Constraint::Type::InequalityPenalty ){
				for(int i = 0; i < n; i++){
					L[k] += 0.5 * square(subcost->con->weight[i]*subcost->b[i]);
				}
			}
			if( subcost->con->type == Constraint::Type::InequalityBarrier ){
				// assume n = 1
				L[k] = -square(subcost->con->weight[0])*log(subcost->b[0]);
			}

		}
	}

	status.timeCost = timer2.CountUS();
	//DSTR << " tL: " << tL << endl;
}

void Solver::CalcCostGradientDDP(){
	for(int k = 0; k <= N; k++){
		vec_clear(Lx [k]);
		mat_clear(Lxx[k]);
	}
	for(int k = 0; k < N; k++){
		vec_clear(Lu [k]);
		mat_clear(Luu[k]);
		mat_clear(Lux[k]);
	}

	timer2.CountUS();
	// calculate state cost
#pragma omp parallel for
	for(int k = 0; k <= N; k++){
		if(!cost[k])
			continue;

		for(SubCost* subcost : cost[k]->subcost){     
            if(!subcost->con->enabled)
				continue;
			if(!subcost->con->active)
				continue;

			int n  = subcost->con->nelem;

			// calc Lx
			for(SubStateLink& x : subcost->x){
				if(x.x->var->locked)
					continue;

				//int j0 = x->var->index;
				int m = x.x->var->nelem;
				x.link->RegisterCoef(x.A, 0, 0, subcost->con->weight);

				if( subcost->con->type == Constraint::Type::Equality ||
					subcost->con->type == Constraint::Type::InequalityPenalty ){
					// Lx = A^T y
					for(int j = 0; j < m; j++){
						for(int i = 0; i < n; i++){
							Lx[k](x.x->index+j) += x.A[i][j]*(subcost->con->weight[i]*subcost->b[i]);
						}
					}
				}
				if( subcost->con->type == Constraint::Type::InequalityBarrier ){
					for(int j = 0; j < m; j++){
						Lx[k](x.x->index+j) += x.A[0][j]*(-subcost->con->weight[0]/subcost->b[0]);
					}
				}
			}

			// calc Lxx
			for(SubStateLink& x0 : subcost->x)for(SubStateLink& x1 : subcost->x){
				if(x0.x->var->locked || x1.x->var->locked)
					continue;

				int m0  = x0.x->var->nelem;
				int m1  = x1.x->var->nelem;

				// Lxx = A^T A
				for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
					for(int i = 0; i < n; i++){
						Lxx[k](x0.x->index+j0, x1.x->index+j1) += x0.A[i][j0]*x1.A[i][j1];
					}
				}

				if( subcost->con->type == Constraint::Type::InequalityBarrier ){
					real_t tmp = 1.0/square(subcost->b[0]);
					for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
						Lxx[k](x0.x->index+j0, x1.x->index+j1) *= tmp;
					}
				}
			}

			if(k < N){
				// calc Lu
				for(SubInputLink& u : subcost->u){
					if(u.u->var->locked)
						continue;

					//int j0 = u->var->index;
					int m  = u.u->var->nelem;
					u.link->RegisterCoef(u.A, 0, 0, subcost->con->weight);

					if( subcost->con->type == Constraint::Type::Equality ||
						subcost->con->type == Constraint::Type::InequalityPenalty ){
						for(int j = 0; j < m; j++){
							for(int i = 0; i < n; i++){
								Lu[k](u.u->index+j) += u.A[i][j]*(subcost->con->weight[i]*subcost->b[i]);
							}
						}
					}
					if( subcost->con->type == Constraint::Type::InequalityBarrier ){
						for(int j = 0; j < m; j++){
							Lu[k](u.u->index+j) += u.A[0][j]*(-subcost->con->weight[0]/subcost->b[0]);
						}
					}
				}

				// calc Luu
				for(SubInputLink& u0 : subcost->u)for(SubInputLink& u1 : subcost->u){
					if(u0.u->var->locked || u1.u->var->locked)
						continue;

					int m0  = u0.u->var->nelem;
					int m1  = u1.u->var->nelem;

					// Lxx = A^T A
					for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
						for(int i = 0; i < n; i++){
							Luu[k](u0.u->index+j0, u1.u->index+j1) += u0.A[i][j0]*u1.A[i][j1];
						}
					}

					//if( subcost->con->type == Constraint::Type::InequalityBarrier ){
					//	real_t tmp = 1.0/square(subcost->b[0]);
					//	for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
					//		Luu[k](u0.u->index+j0, u1.u->index+j1) *= tmp;
					//	}
					//}
				}
				// calc Lux
				for(SubInputLink& u : subcost->u)for(SubStateLink& x : subcost->x){
					if(u.u->var->locked || x.x->var->locked)
						continue;

					int m0  = u.u->var->nelem;
					int m1  = x.x->var->nelem;

					// Lxx = A^T A
					for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
						for(int i = 0; i < n; i++){
							Lux[k](u.u->index+j0, x.x->index+j1) += u.A[i][j0]*x.A[i][j1];
						}
					}

					if( subcost->con->type == Constraint::Type::InequalityBarrier ){
						real_t tmp = 1.0/square(subcost->b[0]);
						for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
							Lux[k](u.u->index+j0, x.x->index+j1) *= tmp;
						}
					}
				}
			}            
		}

        // weights
		for(SubState* subst : state[k]->substate){
            if(subst->var->locked)
                continue;

            int j0 = subst->index;
            int m  = subst->var->nelem;
            for(int j = 0; j < m; j++){
                real_t wj = subst->var->weight[j];

				Lxx[k](j0+j, j0+j) += wj*wj;
            }
        }
        if(k < N){
            for(SubInput* subin : input[k]->subinput){
                if(subin->var->locked)
                    continue;

                int j0 = subin->index;
                int m  = subin->var->nelem;
                for(int j = 0; j < m; j++){
                    real_t wj = subin->var->weight[j];

					Luu[k](j0+j, j0+j) += wj*wj;
                }
            }
        }
	}
	status.timeCostGrad = timer2.CountUS();

	//DSTR << " tLgrad: " << tLgrad << endl;
	/*
	FILE* file = fopen("Luu.csv", "w");
	int k = 5;
	for(int i = 0; i < Luu[k].m; i++){
		for(int j = 0; j < Luu[k].n; j++){
			fprintf(file, "%f, ", Luu[k](i,j));
		}
		fprintf(file, "\n");
	}
	fclose(file);

	file = fopen("Lux.csv", "w");
	for(int i = 0; i < Lux[k].m; i++){
		for(int j = 0; j < Lux[k].n; j++){
			fprintf(file, "%f, ", Lux[k](i,j));
		}
		fprintf(file, "\n");
	}
	fclose(file);
	*/
}

void PrintSparsity(int k, const Matrix& m){
	FILE* file;
	char filename [256];
	sprintf(filename, "quu_%d.csv", k);
	file = fopen(filename, "w");

	const real_t eps = 1.0e-10;

	for(int i = 0; i < m.m; i++){
		for(int j = 0; j < m.n; j++){
			fprintf(file, "%d", (std::abs(m(i,j)) < eps ? 0 : 1));
			if(j != m.n-1)
				fprintf(file, ", ");
		}
		fprintf(file, "\n");
	}

	fclose(file);
}

void Solver::BackwardDDP(){
	int tback1, tback2, tback3;

 	V  [N] = L  [N];
	vec_copy(Lx [N], Vx [N]);
	mat_copy(Lxx[N], Vxx[N]);

	for(int k = N-1; k >= 0; k--){
		timer2.CountUS();
		mat_vec_mul(Vxx[k+1], fcor[k], Vxx_fcor[k], 1.0, 0.0);

		if(state[k]->dim != 0)
			mat_mat_mul(Vxx[k+1], fx  [k], Vxx_fx  [k], 1.0, 0.0);

		if(input[k]->dim != 0)
			mat_mat_mul(Vxx[k+1], fu  [k], Vxx_fu  [k], 1.0, 0.0);

        vec_copy(Vx[k+1]    , Vx_plus_Vxx_fcor[k]);
        vec_add (Vxx_fcor[k], Vx_plus_Vxx_fcor[k]);

        Q[k] = L[k] + V[k+1] + vec_dot(Vx[k+1], fcor[k]) + (1.0/2.0)*vec_dot(fcor[k], Vxx_fcor[k]);
        
        if(state[k]->dim != 0){
			vec_copy(Lx[k], Qx[k]);
			mattr_vec_mul(fx[k], Vx_plus_Vxx_fcor[k], Qx[k], 1.0, 1.0);
		}

        if(input[k]->dim != 0){
			vec_copy(Lu[k], Qu[k]);
			mattr_vec_mul(fu[k], Vx_plus_Vxx_fcor[k], Qu[k], 1.0, 1.0);
		}

        if(state[k]->dim != 0){
			mat_copy(Lxx[k], Qxx[k]);
	        mattr_mat_mul(fx[k], Vxx_fx[k], Qxx[k], 1.0, 1.0);
		}

        if(input[k]->dim != 0){
			mat_copy(Luu[k], Quu[k]);
			mattr_mat_mul(fu[k], Vxx_fu[k], Quu[k], 1.0, 1.0);
		}

        if(input[k]->dim != 0 && state[k]->dim != 0){
			mat_copy(Lux[k], Qux[k]);
	        mattr_mat_mul(fu[k], Vxx_fx[k], Qux[k], 1.0, 1.0);
		}

		//Q  [k] = L  [k] + V[k+1] + Vx[k+1]*f_cor[k] + (1.0/2.0)*((Vxx[k+1]*f_cor[k])*f_cor[k]);
    	//Qx [k] = Lx [k] + fx[k].trans()*(Vx [k+1] + Vxx[k+1]*f_cor[k]);
    	//Qu [k] = Lu [k] + fu[k].trans()*(Vx [k+1] + Vxx[k+1]*f_cor[k]);
    	//Qxx[k] = Lxx[k] + fx[k].trans()*Vxx[k+1]*fx[k];
    	//Quu[k] = Luu[k] + fu[k].trans()*Vxx[k+1]*fu[k];
    	//Qux[k] = Lux[k] + fu[k].trans()*Vxx[k+1]*fx[k];
    	
		for(int i = 0; i < Quu[k].m; i++)
		    Quu[k](i,i) += param.regularization;

		//PrintSparsity(k, Quu[k]);
    	tback1 = timer2.CountUS();

		// input dimension could be zero
		if(Quu[k].m > 0){
			timer2.CountUS();
            mat_inv_pd(Quu[k], Quuinv[k]);
			tback2 = timer2.CountUS();
        
			timer2.CountUS();
			symmat_vec_mul(Quuinv[k], Qu[k], Quuinv_Qu[k], 1.0, 0.0);
			//Quuinv_Qu = Quuinv*Qu;

			V[k] = Q[k] - (1.0/2.0)*vec_dot(Qu[k], Quuinv_Qu[k]);
        
			vec_copy(Qx[k], Vx[k]);
			if(state[k]->dim != 0)
				mattr_vec_mul(Qux[k], Quuinv_Qu[k], Vx[k], -1.0, 1.0);
	    
			mat_copy(Qxx[k], Vxx[k]);
			if(state[k]->dim != 0){
				symmat_mat_mul(Quuinv[k], Qux[k], Quuinv_Qux[k], 1.0, 0.0);
				mattr_mat_mul(Qux[k], Quuinv_Qux[k], Vxx[k], -1.0, 1.0);
			}
			tback3 = timer2.CountUS();
			//mat_inv_sym(Quu[k], Quuinv[k]);
			//
			//Quuinv_Qu[k] = Quuinv[k]*Qu[k];
			//
			//V  [k] = Q  [k] - (1.0/2.0)*(Qu[k]*Quuinv_Qu[k]);
			//Vx [k] = Qx [k] - Qux[k].trans()*Quuinv_Qu [k];
			//Vxx[k] = Qxx[k] - Qux[k].trans()*Quuinv[k]*Qux[k];
		}
		else if(state[k]->dim != 0){
			V  [k] = Q  [k];
			vec_copy(Qx [k], Vx [k]);
			mat_copy(Qxx[k], Vxx[k]);
		}
				
        // enforce symmetry of Uxx
	    for(int i = 1; i < Vxx[k].m; i++) for(int j = 0; j < i; j++)
		    Vxx[k](i,j) = Vxx[k](j,i);

		//int tback = timer2.CountUS();
		//DSTR << "back " << k << " : " << " nx: " << state[k]->dim << " nu: " << input[k]->dim
		//	 << " T1: " << tback1
		//	 << " T2: " << tback2
		//	 << " T3: " << tback3
		//	 << endl;
		//DSTR << "Vk: " << V[k] << endl;
		/*
		char filename[256];
		sprintf(filename, "Quu%d.csv", k);
		FILE* file = fopen(filename, "w");
		for(int i = 0; i < Quu[k].m; i++){
			for(int j = 0; j < Quu[k].n; j++){
				fprintf(file, "%f, ", Quu[k](i,j));
			}
			fprintf(file, "\n");
		}
		fclose(file);
		*/
	}
}

void Solver::BackwardDDPContinuous(){
 	V  [N] = L  [N];
	vec_copy(Lx [N], Vx [N]);
	mat_copy(Lxx[N], Vxx[N]);

	for(int k = N-1; k >= 0; k--){
		timer2.CountUS();

		real_t h = dt[k]/1000;

		// Q
        Q[k] = L[k] + vec_dot(Vx[k+1], fcor[k]);
        
		// Qx
        vec_copy(Lx[k], Qx[k]);
        if(state[k]->dim != 0){
			mattr_vec_mul(fx[k], Vx[k+1], Qx[k], 1.0, 1.0);
			mat_vec_mul  (Vxx[k+1], fcor[k], Qx[k], 1.0, 1.0);
		}
		
		// Qu
        vec_copy(Lu[k], Qu[k]);
        if(input[k]->dim != 0)
			mattr_vec_mul(fu[k], Vx[k+1], Qu[k], 1.0, 1.0);

		// Qxx
        mat_copy(Lxx[k], Qxx[k]);
		if(state[k]->dim != 0){
	        mattr_mat_mul(fx[k], Vxx[k+1], Qxx[k], 1.0, 1.0);
			mat_mat_mul  (Vxx[k+1], fx[k], Qxx[k], 1.0, 1.0);
		}

		// Qux
        mat_copy(Lux[k], Qux[k]);
		if(input[k]->dim != 0 && state[k]->dim != 0)
	        mattr_mat_mul(fu[k], Vxx[k+1], Qux[k], 1.0, 1.0);
    	
		// input dimension could be zero
		if(Quu[k].m > 0){
			// assume diagonal matrix and calc inverse
			mat_clear(Quuinv[k]);
			for(int i = 0; i < Quuinv[k].m; i++)
				Quuinv[k](i,i) = 1.0/Luu[k](i,i);
			
            symmat_vec_mul(Quuinv[k], Qu[k], Quuinv_Qu[k], 1.0, 0.0);

			dV[k] = Q[k] - (1.0/2.0)*vec_dot(Qu[k], Quuinv_Qu[k]);

			vec_copy(Qx[k], dVx[k]);
			if(state[k]->dim != 0)
				mattr_vec_mul(Qux[k], Quuinv_Qu[k], dVx[k], -1.0, 1.0);
						
			mat_copy(Qxx[k], dVxx[k]);
			if(state[k]->dim != 0){
				symmat_mat_mul(Quuinv[k], Qux[k], Quuinv_Qux[k], 1.0, 0.0);
				mattr_mat_mul(Qux[k], Quuinv_Qux[k], dVxx[k], -1.0, 1.0);
			}
		}
		else{
			dV  [k] = Q  [k];
			vec_copy(Qx [k], dVx [k]);
			mat_copy(Qxx[k], dVxx[k]);
		}

		V[k] = V[k+1] + dV[k]*h;

		for(int i = 0; i < Vx[k].n; i++)
			Vx[k](i) = Vx[k+1](i) + dVx[k](i)*h;

		for(int i = 0; i < Vxx[k].m; i++)for(int j = 0; j < Vxx[k].n; j++)
			Vxx[k](i,j) = Vxx[k+1](i,j) + dVxx[k](i,j)*h;

        // enforce symmetry of Uxx
	    for(int i = 1; i < Vxx[k].m; i++) for(int j = 0; j < i; j++)
		    Vxx[k](i,j) = Vxx[k](j,i);

		int tback = timer2.CountUS();
	}
}

void Solver::ForwardDDP(real_t alpha){
    // if the dimension of x0 is not zero, dx0 is also optimized
	if(state[0]->dim == 0){
 		vec_clear(dx[0]);
	}
	else{
		mat_inv_pd(Vxx[0], Vxxinv);
		symmat_vec_mul(Vxxinv, Vx[0], dx[0], -alpha, 0.0);
    
		//dx[0] = -Vxx[0].inv()*Vx[0];
	}

    for(int k = 0; k < N; k++){
		timer2.CountUS();
		if(input[k]->dim != 0){
			vec_copy(Qu[k], Qu_plus_Qux_dx[k]);
			for(int i = 0; i < Qu_plus_Qux_dx[k].n; i++)
				Qu_plus_Qux_dx[k](i) *= alpha;

			if(state[k]->dim != 0)
				mat_vec_mul(Qux[k], dx[k], Qu_plus_Qux_dx[k], 1.0, 1.0);

			symmat_vec_mul(Quuinv[k], Qu_plus_Qux_dx[k], du[k], -1.0, 0.0);
		}

        vec_copy   (fcor[k], dx[k+1]);

		if(state[k]->dim != 0)
			mat_vec_mul(fx  [k], dx[k], dx[k+1], 1.0, 1.0);

		if(input[k]->dim != 0)
			mat_vec_mul(fu  [k], du[k], dx[k+1], 1.0, 1.0);

		//du[k] = -Quuinv[k]*(Qu[k] + Qux[k]*dx[k]);
		//dx[k+1] = fx[k]*dx[k] + fu[k]*du[k] + f_cor[k];

		int tfor = timer2.CountUS();
		//DSTR << "dx[k]: " << dx[k] << endl;
		//DSTR << "for  " << k << " : " << tfor << endl;
	}
}

void Solver::ForwardDDPContinuous(real_t alpha){
    // if the dimension of x0 is not zero, dx0 is also optimized
	if(state[0]->dim == 0){
 		vec_clear(dx[0]);
	}
	else{
		mat_inv_pd(Vxx[0], Vxxinv);
		symmat_vec_mul(Vxxinv, Vx[0], dx[0], -alpha, 0.0);
	}

    for(int k = 0; k < N; k++){
		timer2.CountUS();
		if(input[k]->dim != 0){
			vec_copy(Qu[k], Qu_plus_Qux_dx[k]);
			for(int i = 0; i < Qu_plus_Qux_dx[k].n; i++)
				Qu_plus_Qux_dx[k](i) *= alpha;

			if(state[k]->dim != 0)
				mat_vec_mul(Qux[k], dx[k], Qu_plus_Qux_dx[k], 1.0, 1.0);

			symmat_vec_mul(Quuinv[k], Qu_plus_Qux_dx[k], du[k], -1.0, 0.0);
		}

        vec_copy   (fcor[k], dx[k+1]);

		if(state[k]->dim != 0)
			mat_vec_mul(fx  [k], dx[k], dx[k+1], 1.0, 1.0);

		if(input[k]->dim != 0)
			mat_vec_mul(fu  [k], du[k], dx[k+1], 1.0, 1.0);

		for(int i = 0; i < dx[k+1].n; i++)
			dx[k+1](i) *= dt[k];

		vec_add(dx[k], dx[k+1]);

		int tfor = timer2.CountUS();
	}
}

void Solver::CalcDirectionDDP(){

    CalcTransitionDDP();
	CalcCostDDP();
	CalcCostGradientDDP();

	timer.CountUS();
	if(param.methodMajor == Method::Major::DDPContinuous)
	 	 BackwardDDPContinuous();
	else BackwardDDP();
	status.timeBack = timer.CountUS();
}

real_t Solver::CalcObjectiveDDP(){
	CalcCostDDP();

	real_t Lsum = 0.0;
	for(int k = 0; k <= N; k++)
		Lsum += L[k];

	return Lsum;
}

void Solver::ModifyVariablesDDP(real_t alpha){
    if(param.methodMajor == Method::Major::DDPContinuous)
		 ForwardDDPContinuous(alpha);
	else ForwardDDP(alpha);
    
	for(int k = 0; k <= N; k++){
		for(SubState* subst : state[k]->substate){
			if(subst->var->locked)
				continue;

			int j0 = subst->index;
			for(int j = 0; j < subst->var->nelem; j++){
				subst->var->dx[j] = dx[k](j0+j);
			}
		}
	}
	for(int k = 0; k < N; k++){
		for(SubInput* subin : input[k]->subinput){
			if(subin->var->locked)
				continue;

			int j0 = subin->index;
			for(int j = 0; j < subin->var->nelem; j++){
				subin->var->dx[j] = du[k](j0+j);
			}
		}	
	}
}

}
