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
			};
		};
		struct Minor{
			enum{
				Jacobi,
				GaussSeidel,
			};
		};
	};
	
	struct Param{
		bool    verbose;
		int     methodMajor;
		int     methodMinor;
		int     numIterMajor;  ///< メジャループの反復回数
		int     numIterMinor;  ///< マイナループの反復回数
		real_t  minStepSize;
		real_t  maxStepSize;
		real_t  cutoffStepSize;

		Param();
	};

	struct State{
		real_t  obj;        ///< 評価値
		real_t  objDiff;    ///< 評価値の変化量
		real_t  stepSize;   ///< ステップ幅
		int     iterCount;  ///< 累計反復回数

		State();
	};

	Param           param;
	State           state;
	
	VariableRefs    vars;			///< array of all variables
	ConstraintRefs  cons;			///< array of all constraints
	Constraints     cons_active;	///< array of active constraints
	LinkRefs        links;			///< array of links

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
	/// do initialization
	void Init();

	/// one step update
	void Step();

	/// solve
	void Solve();

	/// deletes all variables and constraints
	void Clear();

	Solver();

};

}
