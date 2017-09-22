#pragma once

#include <sbtypes.h>
#include <sbvariable.h>
#include <sbconstraint.h>
#include <sblink.h>

namespace Scenebuilder{;

/** Solver
    - 最適化問題と拘束解決問題のソルバ
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
				GaussNewton1,   ///< ガウスニュートン　拘束二乗誤差最小化
				GaussNewton2,   ///< 拘束誤差の全成分を一定レートで減少
				Prioritized,
			};
		};
		struct Minor{
			enum{
				Direct,
				Jacobi,
				GaussSeidel,
			};
		};
	};
	
	struct Param{
		bool           verbose;
		int            methodMajor;
		int            methodMinor;
		vector<int>    numIter;       ///< マイナループの反復回数
		real_t         minStepSize;
		real_t         maxStepSize;
		real_t         cutoffStepSize;
		bool           hastyStepSize;

		Param();
	};

	struct State{
		real_t  obj;        ///< 評価値
		real_t  objDiff;    ///< 評価値の変化量
		real_t  stepSize;   ///< ステップ幅
		int     iterCount;  ///< 累計反復回数
		int     timeDir;
		int     timeStep;

		State();
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
				SetCorrectionRate,
			};
		};

		int    type;
		ID     mask;
		bool   enable;
		bool   lock;
		int    level;
		real_t rate;
		real_t lim;
	};

	Param            param;
	State            state;
	vector<Request>  requests;
	
	VariableRefs         vars;			///< array of all variables
	Variables            vars_unlocked;
	ConstraintRefs       cons;			///< array of all constraints
	Constraints          cons_active;	///< array of active constraints
	vector<Constraints>  cons_level;
	
	LinkRefs        links;			///< array of links

	vmat_t          J, JtrJ;
	vvec_t          y, y2;
	vvec_t          Jtry;
	vvec_t          dx;

	vector<ConstraintInfo>  infoType;		///< sum for each constraint category
	vector<ConstraintInfo>  infoLevel;		///< sum for each priority level

	bool  ready;

public:
	/// 内部関数
	real_t  CalcStepSize        ();
	real_t  CalcUpdatedObjective(real_t alpha);
	
	void AddVar   (Variable* var);      ///< add variable
	void DeleteVar(Variable* var);	    ///< delete variable
	void AddCon   (Constraint* con);    ///< add constraint
	void DeleteCon(Constraint* con);    ///< delte variable


public:
	/// 実装すべき仮想関数

	/// 評価関数の値を計算する
	virtual real_t  CalcObjective();

	/// 変数に対する評価関数の微係数を計算する（最急降下法の場合）
	virtual void    CalcDirection();

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
	void SetCorrectionRate(ID mask, real_t rate, real_t lim = FLT_MAX);
	
	/** @brief calculate constraint error
		@param mask			constraint id mask
		@param sum_or_max	if true, sum of constraint errors is returned. otherwise the maximum is returned.
		@param abs_or_rel	if true, absolute error is calculated. otherwise relative error is calculated.

		idに合致する拘束について，abs_or_relにしたがって絶対誤差あるいは相対誤差を計算し，
		sum_or_maxにしたがってそれらの総和あるいは最大値を返す．

		ただし絶対誤差とは誤差ベクトルのノルム，相対誤差とは誤差ベクトルのノルムを，拘束される変数のノルムの和で割ったもの．
	 */
	real_t CalcError(ID mask, bool sum_or_max);

	/// do initialization
	void Init();

	/// one step update
	void Step();

	/// solve
	void Solve();

	/// deletes all variables and constraints
	void Clear();

	void Reset();

	Solver();

};

}
