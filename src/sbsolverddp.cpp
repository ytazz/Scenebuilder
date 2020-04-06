#include <sbsolver.h>
#include <sbmessage.h>

#define USE_MKL

#if defined USE_MKL
# ifdef _WIN32
#  include <mkl_lapacke.h>
# else
#  include <lapacke.h>
# endif
#endif

#include <Foundation/UTQPTimer.h>
static UTQPTimer timer;
static UTQPTimer timer2;

namespace Scenebuilder{;

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

void Solver::AddStateVar(Variable* var, int k){
	if(state.size() <= k)
		state.resize(k+1);

	if(!state[k])
		state[k] = new State();

	SubState* subst = new SubState();
	subst->var = var;
	state[k]->substate.push_back(subst);
}

void Solver::AddInputVar(Variable* var, int k){
	if(input.size() <= k)
		input.resize(k+1);

	if(!input[k])
		input[k] = new Input();

	SubInput* subin = new SubInput();
	subin->var = var;
	input[k]->subinput.push_back(subin);
}

void Solver::AddTransitionCon(Constraint* con, int k){
	if(transition.size() <= k)
		transition.resize(k+1);

	if(!transition[k])
		transition[k] = new Transition();

	State* st_x0 = state[k+0];
	State* st_x1 = state[k+1];
	Input* in_u  = input[k+0];

	SubTransition* subtr = new SubTransition();
	subtr->con = con;
	//subtr->x0 = st_x0->Find(var_x0);
	//subtr->x1 = st_x1->Find(var_x1);
	//subtr->u  = in_u ->Find(var_u );
	transition[k]->subtran.push_back(subtr);
}

void Solver::AddStateCostCon (Constraint* con, int k){
	if(stateCost.size() <= k)
		stateCost.resize(k+1);

	if(!stateCost[k])
		stateCost[k] = new StateCost();

	SubStateCost* subcost = new SubStateCost();
	subcost->con = con;

	stateCost[k]->subcost.push_back(subcost);
}

void Solver::AddInputCostCon (Constraint* con, int k){
	if(inputCost.size() <= k)
		inputCost.resize(k+1);

	if(!inputCost[k])
		inputCost[k] = new InputCost();

	SubInputCost* subcost = new SubInputCost();
	subcost->con = con;

	inputCost[k]->subcost.push_back(subcost);
}

void Solver::InitDDP(){
	N = (int)state.size()-1;

	for(State* st : state){
		st->dim = 0;
		for(SubState* subst : st->substate){
			subst->index = st->dim;
			st->dim += subst->var->nelem;
		}
	}

	for(Input* in : input){
		in->dim = 0;
		for(SubInput* subin : in->subinput){
			subin->index = in->dim;
			in->dim += subin->var->nelem;
		}
	}

	for(Transition* tr : transition){
		
	}

	dx .resize(N+1);
	du .resize(N);
	fx .resize(N);
	fu .resize(N);
	L  .resize(N+1);
	Lx .resize(N+1);
	Lxx.resize(N+1);
	Q  .resize(N);
	Qx .resize(N);
	Qu .resize(N);
	Qxx.resize(N);
	Quu.resize(N);
	Qux.resize(N);
	V  .resize(N+1);
	Vx .resize(N+1);
	Vxx.resize(N+1);

}

void Solver::PrepareDDP(){
	// calculate A and b as whole equation
	CalcEquation();

	for(int k = 0; k < N; k++){
		Transition* tr = transition[k];
		for(SubTransition* subtr : tr->subtran){
			// evaluate f
			subtr->con->CalcLhs();

			int i0 = subtr->con->index;
			int n  = subtr->con->nelem;			

			// set fx
			for(SubState* x0 : subtr->x0){
				int j0 = x0->var->index;
				int m  = x0->var->nelem;
				for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
					fx[k][subtr->x1->index+i][x0->index+j] = -A[i0+i][j0+j];
			}

			// set fu
			for(SubInput* u : subtr->u){
				int j0 = u->var->index; 
				int m  = u->var->nelem;
				for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
					fu[k][subtr->x1->index+i][u->index+j] = -A[i0+i][j0+j];
			}

		}
	}

	for(int k = 0; k <= N; k++){
		L  [k] = 0.0;
		Lx [k].clear();
		Lxx[k].clear();
		Lu [k].clear();
		Luu[k].clear();
	}
	
	// calculate state cost
	for(int k = 0; k <= N; k++){
		for(SubStateCost* subcost : stateCost[k]->subcost){
			int i0 = subcost->con->index;
			int n  = subcost->con->nelem;

			// sum up L
			for(int i = 0; i < n; i++){
				L[k] += 0.5 * square(yvec[i0+i]);
			}

			// calc Lx
			for(SubState* x : subcost->x){
				int j0 = x->var->index;
				int m  = x->var->nelem;

				// Lx = A^T y
				for(int j = 0; j < m; j++){
					Lx[k][x->index+j] = 0.0;
					for(int i = 0; i < n; i++){
						Lx[k][x->index+j] += A[i0+i][j0+j]*yvec[i0*i];
					}
				}
			}

			// calc Lxx
			for(SubState* x0 : subcost->x)for(SubState* x1 : subcost->x){
				int j00 = x0->var->index;
				int j01 = x1->var->index;
				int m0  = x0->var->nelem;
				int m1  = x1->var->nelem;

				// Lxx = A^T A
				for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
					Lxx[k][x0->index+j0][x1->index+j1] = 0.0;
					for(int i = 0; i < n; i++){
						Lxx[k][x0->index+j0][x1->index+j1] += A[i0+i][j00+j0]*A[i0+i][j01+j1];
					}
				}
			}
		}
	}

	// calculate input cost
	for(int k = 0; k < N; k++){
		for(SubInputCost* subcost : inputCost[k]->subcost){
			int i0 = subcost->con->index;
			int n  = subcost->con->nelem;
			
			// sum up L
			for(int i = 0; i < n; i++){
				L[k] += 0.5 * square(yvec[i0+i]);
			}

			// calc Lu
			for(SubInput* u : subcost->u){
				int j0 = u->var->index;
				int m  = u->var->nelem;

				for(int j = 0; j < m; j++){
					Lu[k][u->index+j] = 0.0;
					for(int i = 0; i < n; i++){
						Lu[k][u->index+j] += A[i0+i][j0+j]*yvec[i0*i];
					}
				}
			}

			// calc Luu
			for(SubInput* u0 : subcost->u)for(SubInput* u1 : subcost->u){
				int j00 = u0->var->index;
				int j01 = u1->var->index;
				int m0  = u0->var->nelem;
				int m1  = u1->var->nelem;

				// Lxx = A^T A
				for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
					Luu[k][u0->index+j0][u1->index+j1] = 0.0;
					for(int i = 0; i < n; i++){
						Luu[k][u0->index+j0][u1->index+j1] += A[i0+i][j00+j0]*A[i0+i][j01+j1];
					}
				}
			}
		}

	}
}

void Solver::CalcDirectionDDP(){
	PrepareDDP();

	V  [N] = L  [N];
	Vx [N] = Lx [N];
	Vxx[N] = Lxx[N];

	for(int k = N; k >= 0; k--){
		Qx [k] = Lx [k] + fx[k].trans()*Vx [k+1];
		Qu [k] = Lu [k] + fu[k].trans()*Vx [k+1];
		Qxx[k] = Lxx[k] + fx[k].trans()*Vxx[k+1]*fx[k];
		Quu[k] = Luu[k] + fu[k].trans()*Vxx[k+1]*fu[k];
		Qux[k] =          fu[k].trans()*Vxx[k+1]*fx[k];

		Quu_inv   [k] = inv(Quu[k]);
		Quu_inv_Qu[k] = Quu_inv[k]*Qu[k];

		V  [k] = Q  [k] - (1.0/2.0)*(Qu[k]*Quu_inv_Qu[k]);
		Vx [k] = Qx [k] - Qux[k].trans()*Quu_inv_Qu [k];
		Vxx[k] = Qxx[k] - Qux[k].trans()*Quu_inv[k]*Qux[k];
	}

	dx[0].clear();

	for(int k = 0; k < N; k++){
		du[k]   = -Quu_inv[k]*(Qu[k] + Qux[k]*dx[k]);
		dx[k+1] = fx[k]*dx[k] + fu[k]*du[k];
	}

	for(int k = 0; k < N; k++){
		for(SubInput* subin : input[k]->subinput){
			int j0 = subin->index;
			for(int j = 0; j < subin->var->nelem; j++){
				subin->var->dx[j] = du[k][j0+j];
			}
		}
	}
}

}
