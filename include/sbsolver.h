#pragma once

#include <sbtypes.h>
#include <sbvariable.h>
#include <sbconstraint.h>
#include <sblink.h>
#include <sbblas.h>
#include <sbtimer.h>

namespace Scenebuilder{;

/** Solver
    - solver for optimization (constraint error minimization) and optimal control problems
 **/

typedef vector< Variable* >				Variables;
typedef vector< UTRef<Variable> >		VariableRefs;
typedef vector< Constraint* >			Constraints;
typedef vector< UTRef<Constraint> >		ConstraintRefs;
typedef vector< UTRef<Link> >			LinkRefs;

class Solver : public UTRefCount{
public:
	struct Method{
		struct Major{
			enum{
				SteepestDescent,
				GaussNewton,   ///< Gauss Newton method
				Prioritized,
				DDP,           ///< Differential Dynamic Programming
				DDPContinuous, ///< DDP, continuous-time version
                Num
			};
		};
		struct Minor{
			enum{
				Direct,        ///< direct method
				Jacobi,        ///< Jacobi iterative method
				GaussSeidel,   ///< Gauss-Seidel iterative method
                Num
			};
		};
		/// routine for solving linear equation (when Minor = Direct)
		struct Lapack{
			enum{
				DGELS,   
				DPOSV,
			};
		};
	};
	
	struct Param{
		bool           verbose;
		int            methodMajor;
		int            methodMinor;
		int            methodLapack;
		vector<int>    numIter;          ///< number of minor iterations
		real_t         minStepSize;
		real_t         maxStepSize;
		real_t         cutoffStepSize;
		bool           hastyStepSize;
		real_t         regularization;
		real_t         stateRegularization;
        real_t         complRelaxation;  ///< relaxation parameter of barrier inequality constraints
		bool           useHessian;
		bool           fixInitialState;  ///< fix initial state in ddp
		bool           fixInitialInput;  ///< fix initial input in ddp
		bool           parallelize;
		bool           enableSparse;     ///< use sparse matrix in ddp
		
		Param();
	};

	struct Status{
		real_t  obj;        ///< value of objective function
		real_t  objDiff;    ///< change of value of objective function
		real_t  stepSize;   ///< step size
		int     iterCount;  ///< cumulative iteration count
		int     timePre;
		int     timeDir;
		int     timeStep;
		int     timeMod;
		int     timeTrans, timeTransRev;
		int     timeCost, timeCostGrad;
		int     timeBack;

		Status();
	};

	struct VariableInfo{
		int  num;
		int  numLocked;

		VariableInfo();
	};

	struct ConstraintInfo{
		int     num;
		int     numEnabled;
		int     numActive;
		real_t  error;

		ConstraintInfo();
	};

	struct Request{
		struct Type{
			enum{
				Enable,
				Lock,
				SetPriority,
				SetCorrection,
				SetConstraintWeight,
				SetVariableWeight,
			};
		};

		int    type;
		ID     mask;
		bool   enable;
		bool   lock;
		int    level;
		real_t rate;
		real_t lim;
		vec3_t weight;
	};

	struct SubState : UTRefCount{
		int                 index;
		Variable*           var;
	};
	struct SubInput : UTRefCount{
		int                 index;
		Variable*           var;
	};
	struct SubStateLink{
		SubState* x;
		Link*     link;
		//Matrix    A;
	};
	struct SubInputLink{
		SubInput* u;
		Link*     link;
		//Matrix    A;
	};
		
	struct SubTransition : UTRefCount{
		Constraint*            con;
		SubState*              x1;
		vector<SubStateLink>   x0;
		vector<SubInputLink>   u;
		//Vector                 b;
	};
	struct SubReverseTransition : UTRefCount{
		Constraint*            con;
		SubState*              x0;
		vector<SubStateLink>   x1;
		vector<SubInputLink>   u;
		//Vector                 b;
	};
	struct SubCost : UTRefCount{
		int                    index;
        Constraint*            con;
		vector<SubStateLink>   x;
		vector<SubInputLink>   u;
		int  xbegin, xend;
		int  ubegin, uend;
		//Vector                 b, wb;
	};

	struct State : UTRefCount{
		int  dim;
		vector< UTRef<SubState> > substate;

		SubState*  Find(Variable* var);
	};
	struct Input : UTRefCount{
		int  dim;
		vector< UTRef<SubInput> > subinput;

		SubInput*  Find(Variable* var);
	};
	struct Transition : UTRefCount{
		vector< UTRef<SubTransition> > subtran;
	};
	struct ReverseTransition : UTRefCount{
		vector< UTRef<SubReverseTransition> > subtran;
	};
	struct Cost : UTRefCount{
		int     dim;
		vector< UTRef<SubCost> >  subcost;
		Vector  y, b;
		Matrix  Ax;
		Matrix  Au;
	};
	
	Param            param;
	Status           status;
	vector<Request>  requests;
	
	VariableRefs         vars;			///< array of all variables
	Variables            vars_unlocked;
	ConstraintRefs       cons;			///< array of all constraints
	Constraints          cons_active;	///< array of active constraints
	vector<Constraints>  cons_level;
	
	LinkRefs        links;			///< array of links

	int         dimvar         ;
	int         dimvar_weighted;
	int         dimcon         ;
	Matrix      A, AtrA;
	Vector      b, b2;
	Vector      Atrb;
	Vector      dxvec;
	vector<int> pivot;

	vector<VariableInfo>    varInfoType;
	vector<ConstraintInfo>  conInfoType;		///< sum for each constraint category
	vector<ConstraintInfo>  conInfoLevel;		///< sum for each priority level

	bool  ready;

	vector< UTRef<State> >             state;
	vector< UTRef<Input> >             input;
	vector< UTRef<Transition> >        transition;
	vector< UTRef<ReverseTransition> > transition_rev;
	vector< UTRef<Cost> >              cost;
	
	int               N;
	vector<real_t>    dt;
	vector<Vector>    dx;
	vector<Vector>    du;
	vector<Matrix>    fx, fx_rev;
	vector<Matrix>    fu, fu_rev;
	vector<Vector>    fcor, fcor_rev;
	vector< vector<Matrix> > fxx, fxx_rev;
    vector< vector<Matrix> > fux, fux_rev;
    vector< vector<Matrix> > fuu, fuu_rev;
    vector<real_t>    L;
	vector<Vector>    Lx;
	vector<Matrix>    Lxx;
	vector<Vector>    Lu;
	vector<Matrix>    Luu;
	vector<Matrix>    Lux;
	vector<real_t>    Q;
	vector<Vector>    Qx;
	vector<Vector>    Qu;
	vector<Matrix>    Qxx;
	vector<Matrix>    Quu;
	vector<Matrix>    Qux;
	vector<Matrix>    Quuinv;
	vector<Vector>    Quuinv_Qu;
	vector<real_t>    V, dV;
	vector<Vector>    Vx, dVx;
	vector<Matrix>    Vxx, dVxx;
	vector<Vector>    Vxx_fcor;
	vector<Matrix>    Vxx_fx, fxtr_Vxx;
	vector<Matrix>    Vxx_fu, futr_Vxx;
    vector<Vector>    Vx_plus_Vxx_fcor;
	vector<Matrix>    Quuinv_Qux;
	vector<Vector>    Qu_plus_Qux_dx;
	Matrix            Vxxinv;

	Timer timer, timer2, timer3, timer4;
	
public:
	/// internal functions
	void    Prepare             ();
	real_t  CalcUpdatedObjective(real_t alpha);
	void    CalcEquation        ();
	void    InitDDP             ();
	void    ClearDDP            ();
	void    CalcTransitionDDP   ();
	void    CalcReverseTransitionDDP();
	void    CalcCostDDP         ();
	void    CalcCostGradientDDP ();
	void    BackwardDDP         ();
	void    BackwardDDPContinuous();
	void    ForwardDDP          (real_t alpha);
	void    ForwardDDPContinuous(real_t alpha);
	void    CalcDirectionDDP    ();
	real_t  CalcObjectiveDDP    ();
	void    ModifyVariablesDDP  (real_t alpha);
	
	void AddVar   (Variable* var);      ///< add variable
	void DeleteVar(Variable* var);	    ///< delete variable
	void AddCon   (Constraint* con);    ///< add constraint
	void DeleteCon(Constraint* con);    ///< delte variable

	/// methods for DDP
	SubState*             AddStateVar            (Variable*   var, int k);  ///< register var as a sub-state at k
	SubInput*             AddInputVar            (Variable*   var, int k);  ///< register var as a sub-input at k
	SubTransition*        AddTransitionCon       (Constraint* con, int k);  ///< register con as a forward transition at k
	SubReverseTransition* AddReverseTransitionCon(Constraint* con, int k);  ///< register con as a reverse transition at k
	SubCost*              AddCostCon             (Constraint* con, int k);  ///< register con as a cost at k
	void                  SetTimeStep            (real_t     _dt , int k);  ///< set timestep of k-th step (for continuous ddp)
    
public:
	/// virtual function that are to be overridden by derived classes

	/// 
	virtual void    ModifyVariables     (real_t alpha);

	/// evaluate objective function
	virtual real_t  CalcObjective();

	/// calculate update direction
	virtual void    CalcDirection();

	/// calculate step size
	virtual real_t  CalcStepSize();
	
public:
	/** @brief	enable or disable constraints
		@param	mask	constraint id mask
		@param	enable	enable or disable
	 */
	void Enable(ID mask, bool enable = true);

	/** @brief  lock or unlock variables
		@param	mask	variable id mask
		@param	lock	lock or unlock
	 */
	void Lock(ID mask, bool lock = true);

	/** @brief	set priority level
		@param	mask	constraint id mask
		@param	level	priority level
	 */
	void SetPriority(ID mask, uint level);

	/** @brief	set correction rate
	 */
	void SetCorrection(ID mask, real_t rate, real_t lim = FLT_MAX);

	/** @brief	set constraint weight
	    @param  mask    constraint id mask
		@param  weight  constraint weight

		Solver minimizes the weighted sum of constraint errors and variable changes.
		This function sets the weight of constraints specified by mask.
	 */
	void SetConstraintWeight(ID mask, real_t weight);
	void SetConstraintWeight(ID mask, vec3_t weight);
	
	/** @brief	set variable weight
	    @param  mask    variable id mask
		@param  weight  variable weight

		Solver minimizes the weighted sum of constraint errors and variable changes.
		This function sets the weight of variables specified by mask.
     */
	void SetVariableWeight  (ID mask, real_t weight);
	void SetVariableWeight  (ID mask, vec3_t weight);
	
	/** @brief calculate constraint error
		@param mask			constraint id mask
		@param sum_or_max	if true, sum of constraint errors is returned. otherwise the maximum is returned.
		
		for constraints with IDs that matches mask, sum of max of constraint error
		is calculated depending on sum_or_max.
	 */
	real_t CalcError(ID mask, bool sum_or_max);

	/// do initialization
	virtual void Init();

	/// one step update
	void Step();

	/// deletes all variables and constraints
	void Clear();

	void Reset();

	Solver();

};

}
