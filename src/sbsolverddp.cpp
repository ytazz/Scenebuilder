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

//#include <Foundation/UTQPTimer.h>
//static UTQPTimer timer;
//static UTQPTimer timer2;

namespace Scenebuilder{;

static const real_t inf = numeric_limits<real_t>::max();

inline real_t square(real_t x){
	return x*x;
}
/*
void mat_inv_gen(const vmat_t& m, vmat_t& minv){
#ifdef USE_MKL
    int n = m.height();
	minv = m;
	vector<int> pivot; pivot.resize(n, 0);
    int ret;
	ret = LAPACKE_dgetrf(LAPACK_COL_MAJOR, n, n, &minv[0][0], n, &pivot[0]);
	ret = LAPACKE_dgetri(LAPACK_COL_MAJOR, n,    &minv[0][0], n, &pivot[0]);
#else
    minv = inv(m);
#endif
}

void mat_inv_sym(const vmat_t& m, vmat_t& minv){
#ifdef USE_MKL
    int n = m.height();
	minv = m;
	LAPACKE_dpotrf(LAPACK_COL_MAJOR, 'U', n, &minv[0][0], n);
	LAPACKE_dpotri(LAPACK_COL_MAJOR, 'U', n, &minv[0][0], n);
	for(int i = 1; i < n; i++) for(int j = 0; j < i; j++)
		minv[i][j] = minv[j][i];
#else
    minv = inv(m);
#endif
	//vmat_t test = minv*m;

}
*/
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
			subtr->x0.push_back(subst);
		}
		subin = in_u->Find(l->var);
		if(subin){
			subtr->u.push_back(subin);
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

    return subcost;
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
	Vxx_fcor .resize(N);
	Vxx_fx   .resize(N);
	Vxx_fu   .resize(N);
	Vx_plus_Vxx_fcor.resize(N);
	Quuinv_Qux      .resize(N);
	Qu_plus_Qux_dx  .resize(N);

	for(int k = 0; k <= N; k++){
		int nx  = state[k]->dim;

		dx [k].Allocate(nx);
		Lx [k].Allocate(nx);
		Lxx[k].Allocate(nx, nx);
		Vx [k].Allocate(nx);
		Vxx[k].Allocate(nx, nx);
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

void Solver::PrepareDDP(){
	// calculate A and b as whole equation
	CalcEquation();

	for(int k = 0; k < N; k++){
		Transition* tr = transition[k];

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
            
            // set fx
			for(SubState* x0 : subtr->x0){
				if(x0->var->locked)
					continue;

				int j0 = x0->var->index;
				int m  = x0->var->nelem;
				for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
					fx[k](subtr->x1->index+i, x0->index+j) = -A[i0+i][j0+j];
			}
            
            // set fu
			for(SubInput* u : subtr->u){
				if(u->var->locked)
					continue;

				int j0 = u->var->index; 
				int m  = u->var->nelem;
				for(int i = 0; i < n; i++)for(int j = 0; j < m; j++)
					fu[k](subtr->x1->index+i, u->index+j) = -A[i0+i][j0+j];
			}
            
            // set correction term
			for(int i = 0; i < n; i++)
				fcor[k](subtr->x1->index+i) = b[i0+i];

		}
	}

    for(int k = 0; k <= N; k++){
		L  [k] = 0.0;
		vec_clear(Lx [k]);
		mat_clear(Lxx[k]);
	}
	for(int k = 0; k < N; k++){
		vec_clear(Lu [k]);
		mat_clear(Luu[k]);
		mat_clear(Lux[k]);
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
				//L[k] += 0.5 * square(yvec[i0+i]);
				L[k] += 0.5 * square(b[i0+i]);
			}

			// calc Lx
			for(SubState* x : subcost->x){
				if(x->var->locked)
					continue;

				int j0 = x->var->index;
				int m  = x->var->nelem;

				// Lx = A^T y
				for(int j = 0; j < m; j++){
					for(int i = 0; i < n; i++){
						//Lx[k](x->index+j) += A[i0+i][j0+j]*yvec[i0+i];
						Lx[k](x->index+j) += A[i0+i][j0+j]*(-b[i0+i]);
					}
				}
			}

			// calc Lxx
			for(SubState* x0 : subcost->x)for(SubState* x1 : subcost->x){
				if(x0->var->locked || x1->var->locked)
					continue;

				int j00 = x0->var->index;
				int j01 = x1->var->index;
				int m0  = x0->var->nelem;
				int m1  = x1->var->nelem;

				// Lxx = A^T A
				for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
					for(int i = 0; i < n; i++){
						Lxx[k](x0->index+j0, x1->index+j1) += A[i0+i][j00+j0]*A[i0+i][j01+j1];
					}
				}
			}

			if(k < N){
				// calc Lu
				for(SubInput* u : subcost->u){
					if(u->var->locked)
						continue;

					int j0 = u->var->index;
					int m  = u->var->nelem;

					for(int j = 0; j < m; j++){
						for(int i = 0; i < n; i++){
							//Lu[k](u->index+j) += A[i0+i][j0+j]*yvec[i0+i];
							Lu[k](u->index+j) += A[i0+i][j0+j]*(-b[i0+i]);
						}
					}
				}

				// calc Luu
				for(SubInput* u0 : subcost->u)for(SubInput* u1 : subcost->u){
					if(u0->var->locked || u1->var->locked)
						continue;

					int j00 = u0->var->index;
					int j01 = u1->var->index;
					int m0  = u0->var->nelem;
					int m1  = u1->var->nelem;

					// Lxx = A^T A
					for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
						for(int i = 0; i < n; i++){
							Luu[k](u0->index+j0, u1->index+j1) += A[i0+i][j00+j0]*A[i0+i][j01+j1];
						}
					}
				}
				// calc Lux
				for(SubInput* u : subcost->u)for(SubState* x : subcost->x){
					if(u->var->locked || x->var->locked)
						continue;

					int j00 = u->var->index;
					int j01 = x->var->index;
					int m0  = u->var->nelem;
					int m1  = x->var->nelem;

					// Lxx = A^T A
					for(int j0 = 0; j0 < m0; j0++)for(int j1 = 0; j1 < m1; j1++){
						for(int i = 0; i < n; i++){
							Lux[k](u->index+j0, x->index+j1) += A[i0+i][j00+j0]*A[i0+i][j01+j1];
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
}

void Solver::BackwardDDP(){
 	V  [N] = L  [N];
	vec_copy(Lx [N], Vx [N]);
	mat_copy(Lxx[N], Vxx[N]);

	for(int k = N-1; k >= 0; k--){
		mat_vec_mul(Vxx[k+1], fcor[k], Vxx_fcor[k], 1.0, 0.0);
        mat_mat_mul(Vxx[k+1], fx  [k], Vxx_fx  [k], 1.0, 0.0);
        mat_mat_mul(Vxx[k+1], fu  [k], Vxx_fu  [k], 1.0, 0.0);
        vec_copy(Vx[k+1]    , Vx_plus_Vxx_fcor[k]);
        vec_add (Vxx_fcor[k], Vx_plus_Vxx_fcor[k]);

        Q[k] = L[k] + V[k+1] + vec_dot(Vx[k+1], fcor[k]) + (1.0/2.0)*vec_dot(fcor[k], Vxx_fcor[k]);
        
        vec_copy(Lx[k], Qx[k]);
        mattr_vec_mul(fx[k], Vx_plus_Vxx_fcor[k], Qx[k], 1.0, 1.0);

        vec_copy(Lu[k], Qu[k]);
        mattr_vec_mul(fu[k], Vx_plus_Vxx_fcor[k], Qu[k], 1.0, 1.0);

        mat_copy(Lxx[k], Qxx[k]);
        mattr_mat_mul(fx[k], Vxx_fx[k], Qxx[k], 1.0, 1.0);

        mat_copy(Luu[k], Quu[k]);
        mattr_mat_mul(fu[k], Vxx_fu[k], Quu[k], 1.0, 1.0);

        mat_copy(Lux[k], Qux[k]);
        mattr_mat_mul(fu[k], Vxx_fx[k], Qux[k], 1.0, 1.0);

		//Q  [k] = L  [k] + V[k+1] + Vx[k+1]*f_cor[k] + (1.0/2.0)*((Vxx[k+1]*f_cor[k])*f_cor[k]);
    	//Qx [k] = Lx [k] + fx[k].trans()*(Vx [k+1] + Vxx[k+1]*f_cor[k]);
    	//Qu [k] = Lu [k] + fu[k].trans()*(Vx [k+1] + Vxx[k+1]*f_cor[k]);
    	//Qxx[k] = Lxx[k] + fx[k].trans()*Vxx[k+1]*fx[k];
    	//Quu[k] = Luu[k] + fu[k].trans()*Vxx[k+1]*fu[k];
    	//Qux[k] = Lux[k] + fu[k].trans()*Vxx[k+1]*fx[k];
    	
		for(int i = 0; i < Quu[k].m; i++)
		    Quu[k](i,i) += param.regularization;
    	
		// input dimension could be zero
		if(Quu[k].m > 0){
            mat_inv_pd(Quu[k], Quuinv[k]);
        
			symmat_vec_mul(Quuinv[k], Qu[k], Quuinv_Qu[k], 1.0, 0.0);
			//Quuinv_Qu = Quuinv*Qu;

			V[k] = Q[k] - (1.0/2.0)*vec_dot(Qu[k], Quuinv_Qu[k]);
        
			vec_copy(Qx[k], Vx[k]);
			mattr_vec_mul(Qux[k], Quuinv_Qu[k], Vx[k], -1.0, 1.0);
	    
			mat_copy(Qxx[k], Vxx[k]);
			symmat_mat_mul(Quuinv[k], Qux[k], Quuinv_Qux[k], 1.0, 0.0);
			mattr_mat_mul(Qux[k], Quuinv_Qux[k], Vxx[k], -1.0, 1.0);
		
			//mat_inv_sym(Quu[k], Quuinv[k]);
			//
			//Quuinv_Qu[k] = Quuinv[k]*Qu[k];
			//
			//V  [k] = Q  [k] - (1.0/2.0)*(Qu[k]*Quuinv_Qu[k]);
			//Vx [k] = Qx [k] - Qux[k].trans()*Quuinv_Qu [k];
			//Vxx[k] = Qxx[k] - Qux[k].trans()*Quuinv[k]*Qux[k];
		}
		else{
			V  [k] = Q  [k];
			vec_copy(Qx [k], Vx [k]);
			mat_copy(Qxx[k], Vxx[k]);
		}
				
        // enforce symmetry of Uxx
	    for(int i = 1; i < Vxx[k].m; i++) for(int j = 0; j < i; j++)
		    Vxx[k](i,j) = Vxx[k](j,i);

	}
}

void Solver::ForwardDDP(){
    // if the dimension of x0 is not zero, dx0 is also optimized
	if(state[0]->dim == 0){
 		vec_clear(dx[0]);

		//dx[0].clear();
	}
	else{
		mat_inv_pd(Vxx[0], Vxxinv);
		symmat_vec_mul(Vxxinv, Vx[0], dx[0], -1.0, 0.0);
    
		//dx[0] = -Vxx[0].inv()*Vx[0];
	}

    for(int k = 0; k < N; k++){
        vec_copy(Qu[k], Qu_plus_Qux_dx[k]);
        mat_vec_mul(Qux[k], dx[k], Qu_plus_Qux_dx[k], 1.0, 1.0);
        symmat_vec_mul(Quuinv[k], Qu_plus_Qux_dx[k], du[k], -1.0, 0.0);

        vec_copy   (fcor[k], dx[k+1]);
        mat_vec_mul(fx  [k], dx[k], dx[k+1], 1.0, 1.0);
        mat_vec_mul(fu  [k], du[k], dx[k+1], 1.0, 1.0);

		//du[k] = -Quuinv[k]*(Qu[k] + Qux[k]*dx[k]);
		//dx[k+1] = fx[k]*dx[k] + fu[k]*du[k] + f_cor[k];
	}
}

void Solver::CalcDirectionDDP(){
    PrepareDDP();

    BackwardDDP();
    ForwardDDP();
    
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

real_t Solver::CalcObjectiveDDP(){
	PrepareDDP();

	real_t Lsum = 0.0;
	for(int k = 0; k <= N; k++)
		Lsum += L[k];

	return Lsum;
}

}
