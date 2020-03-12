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

void Solver::AddTransitionCon(Constraint* con, int k, Variable* var_x1, Variable* var_x0, Variable* var_u){
	if(transition.size() <= k)
		transition.resize(k+1);

	if(!transition[k])
		transition[k] = new Transition();

	State* st_x0 = state[k+0];
	State* st_x1 = state[k+1];
	Input* in_u  = input[k+0];

	SubTransition* subtr = new SubTransition();
	subtr->x0 = st_x0->Find(var_x0);
	subtr->x1 = st_x1->Find(var_x1);
	subtr->u  = in_u ->Find(var_u );
	transition[k]->subtran.push_back(subtr);
}

void Solver::AddStateCostCon (Constraint* con, int k, Variable* var_x){
	if(stateCost.size() <= k)
		stateCost.resize(k+1);

	if(!stateCost[k])
		stateCost[k] = new StateCost();

	SubStateCost* subcost = new SubStateCost();
	subcost->con = con;

	stateCost[k]->subcost.push_back(subcost);
}

void Solver::AddInputCostCon (Constraint* con, int k, Variable* var_u){
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
			int i0    = subtr    ->con->index;
			int j0_x0 = subtr->x0->var->index;
			int j0_u  = subtr->u ->var->index; 
			int n     = subtr    ->con->nelem;
			int m_x0  = subtr->x0->var->nelem;
			int m_u   = subtr->u ->var->nelem;

			// evaluate f
			subtr->con->CalcRhs(subtr->x1->var);

			// set fx
			for(int i = 0; i < n; i++)for(int j = 0; j < m_x0; j++)
				fx[k][subtr->x1->index+i][subtr->x0->index+j] = -A[i0+i][j0_x0+j];

			// set fu
			for(int i = 0; i < n; i++)for(int j = 0; j < m_u; j++)
				fu[k][subtr->x1->index+i][subtr->u->index+j] = -A[i0+i][j0_u+j];
		}
	}

	for(int k = 0; k <= N; k++){
		L[k] = 0.0;
	}
	
	// calculate state cost
	for(int k = 0; k <= N; k++){
		for(SubStateCost* subcost : stateCost[k]->subcost){
			int i0 = subcost   ->con->index;
			int j0 = subcost->x->var->index;
			int n  = subcost   ->con->nelem;
			int m  = subcost->x->var->nelem;

			// sum up L
			for(int i = 0; i < n; i++){
				L[k] += 0.5 * square(yvec[i0+i]);
			}

			// calc Lx
			for(int j = 0; j < m; j++){
				Lx[k][subcost->x->index+j] = 0.0;
				for(int i = 0; i < n; i++){
					Lx[k][subcost->x->index+j] += A[i0+i][j0+j]*yvec[i0*i];
				}
			}

			// calc Lxx
			for(int j1 = 0; j1 < m; j1++)for(int j2 = 0; j2 < m; j2++){
				Lxx[k][subcost->x->index+j1][subcost->x->index+j2] = 0.0;
				for(int i = 0; i < n; i++){
					Lxx[k][subcost->x->index+j1][subcost->x->index+j2] += A[i0+i][j0+j1]*A[i0+i][j0+j2];
				}
			}
		}
	}

	// calculate input cost
	for(int k = 0; k < N; k++){
		for(SubInputCost* subcost : inputCost[k]->subcost){
			int i0 = subcost   ->con->index;
			int j0 = subcost->u->var->index;
			int n  = subcost   ->con->nelem;
			int m  = subcost->u->var->nelem;

			// sum up L
			for(int i = 0; i < n; i++){
				L[k] += 0.5 * square(yvec[i0+i]);
			}

			// calc Lu
			for(int j = 0; j < m; j++){
				Lu[k][subcost->u->index+j] = 0.0;
				for(int i = 0; i < n; i++){
					Lu[k][subcost->u->index+j] += A[i0+i][j0+j]*yvec[i0*i];
				}
			}

			// calc Luu
			for(int j1 = 0; j1 < m; j1++)for(int j2 = 0; j2 < m; j2++){
				Luu[k][subcost->u->index+j1][subcost->u->index+j2] = 0.0;
				for(int i = 0; i < n; i++){
					Luu[k][subcost->u->index+j1][subcost->u->index+j2] += A[i0+i][j0+j1]*A[i0+i][j0+j2];
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
