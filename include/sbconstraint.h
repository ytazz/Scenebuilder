#pragma once

#include <sbvariable.h>
#include <sbblas.h>

#include <map>

using namespace std;
using namespace Spr;

namespace Scenebuilder{;

class Solver;
class SLink;
class C2Link;
class R2Link;
class M2Link;
class X3Link;
class C3Link;
class R3Link;
class M3Link;

/**
	constraint base class
 */
class Constraint : public UTRefCount, public ID{
public:
    struct Type{
        enum{
            Equality,
            InequalityPenalty,
            InequalityBarrier,
        };
    };

	typedef void (Constraint::*UpdateFunc)(uint k, real_t d);

	Solver*		solver;
    int         type;       ///< constraint type
	Links		links;		///< links to constrained variables
	Links       links_active;
	int         nelem;      ///< dimension of this constraint
	int         nelem_var;  ///< total dimension of variables linked with this constraint
	int         level;      ///< priority level
	int         index;      ///< index of this constraint in entire constraint variable y
	bool		enabled;	///< enabled constraint (controlled by user)
	bool		active;		///< active constraint  (for penalty inequality constraints)
	vec3_t      weight;     ///< weight of constraint error (component-wise)
	real_t      barrier_margin;   ///< margin from zero (for barrier inequality)
    real_t		scale, scale2, scale_inv, scale2_inv;		///< scaling coefficient
	vector<vmat_t>  hessian;
	
	/** error correction rate
		誤差修正率．
		(誤差修正量) = (誤差) x (correction rate)
		通常は[0,1)の値を設定する．理論上は(-1,1)であれば誤差は0へ収束する
		実用上，corrRateを2などに設定すると少ない反復で早い収束が得られることもある
		*/
	real_t	corrRate;

	/** maximum error correction
		誤差修正量の上限．
		誤差修正量はこの値を超えないようにクリッピングされる．
		優先度レベルが複数ある場合に定常偏差を低減するにはこの値を小さ目に設定する．
		ただし小さくするほど収束スピードは低下する．
	 */
	real_t	corrMax;

	vec3_t		e;			///< error value
	vec3_t		y;			///< constraint error
	vec3_t		dy;			///< change of constraint error
	vec3_t		dyd;		///< desired change of constraint error
	vec3_t		l;			///< multiplier
	vec3_t		dl;			///< change of multiplier
	vec3_t		J, Jinv;	///< square sum of Jacobian row and its inverse

	/// テスト用
	ofstream	file;

public:	
	SLink*		AddSLink (Variable* var, real_t coef = 1.0);
	C2Link*		AddC2Link(Variable* var);
	R2Link*		AddR2Link(Variable* var);
	M2Link*		AddM2Link(Variable* var);
	X3Link*		AddX3Link(Variable* var);
	C3Link*		AddC3Link(Variable* var);
	R3Link*		AddR3Link(Variable* var);
	M3Link*		AddM3Link(Variable* var);

	void SetPriority(uint newlv);

	/// reset internal variables
	void ResetState();

	/// preparation
	void CalcError();
	void CalcCorrection();
	void RegisterCorrection(Vector&& dydvec, const vec3_t& _weight);
	void RegisterDeviation (Vector&& yvec);

	/// steepest-descent
	void UpdateGradient(uint k);

	/// G-S related routines
	void UpdateMultiplier(uint k);
	void UpdateError1(uint k, real_t ddy);
	void UpdateError2(uint k, real_t _dy);
	void UpdateError3(uint k, real_t ddy);
	void UpdateConjugate(uint k);

public:
	/// virtual functions to be overridden by derived classes ///
	virtual void CalcCoef         (){}
	virtual void CalcDeviation    ();
	virtual void CalcHessian      (){}
	
public:
	/// G-S related virtual functions
	
	/** do projection on multiplier
	 */
	virtual void Project(real_t& l, uint k){}
	
	Constraint(Solver* solver, uint n, ID _id, int type, real_t _scale);
};

/** fixation constraint
 */
struct FixConS : Constraint{
	real_t	desired;
	virtual void CalcDeviation();
	FixConS(Solver* solver, ID id, SVar* var, real_t _scale);
};
struct FixConV2 : Constraint{
	vec2_t	desired;
	virtual void CalcDeviation();
	FixConV2(Solver* solver, ID id, V2Var* var, real_t _scale);
};
struct FixConV3 : Constraint{
	vec3_t	desired;
	virtual void CalcDeviation();
	FixConV3(Solver* solver, ID id, V3Var* var, real_t _scale);
};
struct FixConQ : Constraint{
	quat_t	desired;
	virtual void CalcDeviation();
	FixConQ(Solver* solver, ID id, QVar* var, real_t _scale);
};

/** matching constraint
 */
struct MatchConS : Constraint{
	MatchConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale);
};
struct MatchConV2 : Constraint{
	MatchConV2(Solver* solver, ID id, V2Var* var0, V2Var* var1, real_t _scale);
};
struct MatchConV3 : Constraint{
	MatchConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale);
};
struct MatchConQ : Constraint{
	virtual void CalcDeviation();
	MatchConQ(Solver* solver, ID id, QVar* var0, QVar* var1, real_t _scale);
};


/**	range constraint for scalar variables
 */
struct RangeConS : Constraint{
	real_t	_min, _max;
	bool	on_lower, on_upper;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConS(Solver* solver, ID id, SVar* var, real_t _scale);
};

// 2d/3d range constraint deprecated: activeness cannot be shared by multiple axes
class RangeConV2 : public Constraint{
public:
	real_t	_min, _max;
	bool	on_lower, on_upper;
	vec2_t  dir;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConV2(Solver* solver, ID id, V2Var* var, const vec2_t& _dir, real_t _scale);
};

class RangeConV3 : public Constraint{
public:
	real_t	_min, _max;
	bool	on_lower, on_upper;
	vec3_t  dir;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	RangeConV3(Solver* solver, ID id, V3Var* var, const vec3_t& _dir, real_t _scale);
};

/**	range constraint for difference of scalar variables
    _min <= var1 - var0 <= _max
 */
struct DiffConS : Constraint{
	real_t	_min, _max;
	bool	on_lower, on_upper;

	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);

	DiffConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale);
};

/** general linear inequality constraint of scalar variables
	A0 * x0 + A1 * x1 <= B
 */
/*struct LinearConS : Constraint{
	
};*/

/** fix vector on plane
	n^T (x - o) = 0
	n: normal of plane
	o: origin of plane
 */
class FixConPlane : public Constraint{
public:
	vec3_t	normal;
	vec3_t  origin;
	
	virtual void CalcCoef();
	virtual void CalcDeviation();
	FixConPlane(Solver* solver, ID id, V3Var* var, real_t _scale);
};

class RangeConPlane : public Constraint{
public:
	vec3_t  normal;
	vec3_t  origin;
	real_t  _min, _max;
	bool	on_lower, on_upper;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);
	RangeConPlane(Solver* solver, ID id, V3Var* var, real_t _scale);
};

/** complementarity constraint of scalar variables
	x0 * x1 = 0
 */
class ComplConS : public Constraint{
public:
	virtual void CalcCoef();
	virtual void CalcDeviation();

	ComplConS(Solver* solver, ID id, SVar* var0, SVar* var1, real_t _scale);
};

/** range constraint on euclidean distance
	l <= ||x - y|| <= u
 */
class DistanceConV3 : public Constraint{
public:
	vec3_t	diff;
	real_t	diff_norm;
	real_t  _min, _max;
	bool	on_lower, on_upper;

	virtual void CalcCoef();
	virtual void CalcDeviation();
	virtual void Project(real_t& l, uint k);
	DistanceConV3(Solver* solver, ID id, V3Var* var0, V3Var* var1, real_t _scale);
};

/**
	C0 continuity constraint
 */
class C0ConS : public Constraint{
public:
	real_t h;

	virtual void CalcCoef();
	C0ConS(Solver* solver, ID id, SVar* p0, SVar* v0, SVar* p1, real_t _scale);
};
class C0ConV3 : public Constraint{
public:
	real_t h;

	virtual void CalcCoef();
	C0ConV3(Solver* solver, ID id, V3Var* p0, V3Var* v0, V3Var* p1, real_t _scale);
};

/**
	C1 continuity constraint
 */
class C1ConS : public Constraint{
public:
	real_t h;

	virtual void CalcCoef();
	C1ConS(Solver* solver, ID id, SVar* p0, SVar* v0, SVar* a0, SVar* p1, real_t _scale);
};
class C1ConV3 : public Constraint{
public:
	real_t h;

	virtual void CalcCoef();
	C1ConV3(Solver* solver, ID id, V3Var* p0, V3Var* v0, V3Var* a0, V3Var* p1, real_t _scale);
};

}
