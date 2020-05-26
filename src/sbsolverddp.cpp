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
			subtr->x0.push_back(subst);
		}
		subin = in_u->Find(l->var);
		if(subin){
			subtr->u.push_back(subin);
		}
	}

	transition[k]->subtran.push_back(subtr);
}

void Solver::AddCostCon (Constraint* con, int k){
	if(cost.size() <= k)
		cost.resize(k+1);

	if(!cost[k])
		cost[k] = new Cost();

	State* st_x = state[k];
	Input* in_u = input[k];

	SubCost* subcost = new SubCost();
	subcost->con = con;

	for(Link* l : con->links){
		SubState* subst;
		SubInput* subin;

		subst = st_x->Find(l->var);
		if(subst){
			subcost->x.push_back(subst);
		}
		subin = in_u->Find(l->var);
		if(subin){
			subcost->u.push_back(subin);
		}
	}

	cost[k]->subcost.push_back(subcost);
}

void Solver::InitDDP(){
	N = (int)state.size()-1;

	for(State* st : state){
		st->dim = 0;
		for(SubState* subst : st->substate){
			if(subst->var->locked)
				continue;

			subst->index = st->dim;
			st->dim += subst->var->nelem;
		}
	}

	for(Input* in : input){
		in->dim = 0;
		for(SubInput* subin : in->subinput){
			if(subin->var->locked)
				continue;

			subin->index = in->dim;
			in->dim += subin->var->nelem;
		}
	}

	for(Transition* tr : transition){
		
	}

	dx       .resize(N+1);
	du       .resize(N);
	fx       .resize(N);
	fu       .resize(N);
	f_cor    .resize(N);
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

	for(int k = 0; k <= N; k++){
		int nx  = state[k]->dim;

		dx [k].resize(nx);
		Lx [k].resize(nx);
		Lxx[k].resize(nx, nx);
		Vx [k].resize(nx);
		Vxx[k].resize(nx, nx);

		if(k < N){
			int nu  = input[k]->dim;
			int nx1 = state[k+1]->dim;

			du       [k].resize(nu);
			fx       [k].resize(nx1, nx);
			fu       [k].resize(nx1, nu);
			f_cor    [k].resize(nx1);
			Lu       [k].resize(nu);
			Luu      [k].resize(nu, nu);
			Lux      [k].resize(nu, nx);
			Qx       [k].resize(nx);
			Qu       [k].resize(nu);
			Qxx      [k].resize(nx, nx);
			Quu      [k].resize(nu, nu);
			Quuinv   [k].resize(nu, nu);
			Quuinv_Qu[k].resize(nu);
			Qux      [k].resize(nu, nx);

			DSTR << "k: " << k << " nx: " << nx << " nu: " << nu << " cost: " << cost[k]->subcost.size() << endl;
		}
		else{
			DSTR << "k: " << k << " nx: " << nx << " cost: " << cost[k]->subcost.size() << endl;
		}
	}

}

void Solver::PrepareDDP(){
	// calculate A and b as whole equation
	CalcEquation();

	for(int k = 0; k < N; k++){
		Transition* tr = transition[k];

		fx[k].clear();
		fu[k].clear();

		for(SubTransition* subtr : tr->subtran){
			// evaluate f
			//subtr->con->CalcLhs();

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

			// set correction term
			for(int i = 0; i < n; i++)
				f_cor[k][subtr->x1->index+i] = b[i0+i];

		}
	}

	for(int k = 0; k <= N; k++){
		L  [k] = 0.0;
		Lx [k].clear();
		Lxx[k].clear();
	}
	for(int k = 0; k < N; k++){
		Lu [k].clear();
		Luu[k].clear();
		Lux[k].clear();
	}

	// calculate state cost
	for(int k = 0; k <= N; k++){
		for(SubCost* subcost : cost[k]->subcost){
			if(!subcost->con->enabled)
				continue;
			if(!subcost->con->active)
				continue;

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
					for(int i = 0; i < n; i++){
						Lx[k][x->index+j] += A[i0+i][j0+j]*yvec[i0+i];
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
					for(int i = 0; i < n; i++){
						Lxx[k][x0->index+j0][x1->index+j1] += A[i0+i][j00+j0]*A[i0+i][j01+j1];
					}
				}
			}

			if(k < N){
				// calc Lu
				for(SubInput* u : subcost->u){
					int j0 = u->var->index;
					int m  = u->var->nelem;

					for(int j = 0; j < m; j++){
						for(int i = 0; i < n; i++){
							Lu[k][u->index+j] += A[i0+i][j0+j]*yvec[i0+i];
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
						for(int i = 0; i < n; i++){
							Luu[k][u0->index+j0][u1->index+j1] += A[i0+i][j00+j0]*A[i0+i][j01+j1];
						}
					}
				}
				// calc Lux
				for(SubInput* u : subcost->u)for(SubState* x : subcost->x){
					int j00 = u->var->index;
					int j01 = x->var->index;
					int m0  = u->var->nelem;
					int m1  = x->var->nelem;

					// Lxx = A^T A
					for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
						for(int i = 0; i < n; i++){
							Lux[k][u->index+j0][x->index+j1] += A[i0+i][j00+j0]*A[i0+i][j01+j1];
						}
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

	for(int k = N-1; k >= 0; k--){
		Q  [k] = L  [k] + V[k+1];
		Qx [k] = Lx [k] + fx[k].trans()*(Vx [k+1] + Vxx[k+1]*f_cor[k]);
		Qu [k] = Lu [k] + fu[k].trans()*(Vx [k+1] + Vxx[k+1]*f_cor[k]);
		Qxx[k] = Lxx[k] + fx[k].trans()*Vxx[k+1]*fx[k];
		Quu[k] = Luu[k] + fu[k].trans()*Vxx[k+1]*fu[k];
		Qux[k] = Lux[k] + fu[k].trans()*Vxx[k+1]*fx[k];

		//DSTR << "k: " << k << endl;
		//DSTR << " Quu: " << endl; 
		//DSTR << Quu[k] << endl;
		//DSTR << " Luu: " << endl; 
		//DSTR << Luu[k] << endl;

		const real_t eps = 0.000001;
		int n = Quu[k].width();
		for(int i = 0; i < n; i++)
			Quu[k][i][i] += eps;

#ifdef USE_MKL
		Quuinv[k] = Quu[k];
		LAPACKE_dpotrf(LAPACK_COL_MAJOR, 'U', n, &Quuinv[k][0][0], n);
		LAPACKE_dpotri(LAPACK_COL_MAJOR, 'U', n, &Quuinv[k][0][0], n);
		for(int i = 1; i < n; i++) for(int j = 0; j < i; j++)
			Quuinv[k][i][j] = Quuinv[k][j][i];
#else
		Quuinv[k] = inv(Quu[k]);
#endif

		vmat_t test = Quuinv[k]*Quu[k];

		Quuinv_Qu[k] = Quuinv[k]*Qu[k];

		V  [k] = Q  [k] - (1.0/2.0)*(Qu[k]*Quuinv_Qu[k]);
		Vx [k] = Qx [k] - Qux[k].trans()*Quuinv_Qu [k];
		Vxx[k] = Qxx[k] - Qux[k].trans()*Quuinv[k]*Qux[k];
		
		// enforce symmetry of Vxx
		int nx = Vxx[k].width();
		for(int i = 1; i < nx; i++) for(int j = 0; j < i; j++)
			Vxx[k][i][j] = Vxx[k][j][i];

		//DSTR << "test" << endl;
		//DSTR << Luu[k] << endl;
		//DSTR << Lxx[k] << endl;
		//DSTR << fx[k] << endl;
		//DSTR << fu[k] << endl;
		//DSTR << Lux[k] << endl;
		//DSTR << Qxx[k] << endl;
		//DSTR << Vxx[k] << endl;
		//DSTR << Quu[k] << endl;
		//DSTR << Quuinv[k] << endl;
		//DSTR << test << endl;
	}

	bool fix_x0 = false;
	if(fix_x0){
 		dx[0].clear();
	}
	else{
		dx[0] = -Vxx[0].inv()*Vx[0];
	}

	for(int k = 0; k < N; k++){
		du[k]   = -Quuinv[k]*(Qu[k] + Qux[k]*dx[k]);
		dx[k+1] = fx[k]*dx[k] + fu[k]*du[k] + f_cor[k];

		//DSTR << "k: " << k << " du: " << du[k] << " dx: " << dx[k] << endl;
		//DSTR << "k: " << k << " f_cor: " << f_cor[k] << endl;
	}

	for(int k = 0; k <= N; k++){
		for(SubState* subst : state[k]->substate){
			int j0 = subst->index;
			for(int j = 0; j < subst->var->nelem; j++){
				subst->var->dx[j] = dx[k][j0+j];
			}
		}
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

void Solver::ForwardDynamics(){
	for(int k = 0; k < N; k++){
		Transition* tr = transition[k];
		for(SubTransition* subtr : tr->subtran){
			subtr->con->CalcLhs();
		}
	}
}

real_t Solver::CalcObjectiveDDP(){
	//ForwardDynamics();

	PrepareDDP();

	real_t Lsum = 0.0;
	for(int k = 0; k <= N; k++)
		Lsum += L[k];

	return Lsum;
}

}
